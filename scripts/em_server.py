if __name__ in ('__main__','__mp_main__'):
    from em_session import bootstrap
    bootstrap()

import argparse
import ast
import errno
import hashlib
import json
import math
import orjson
import mimetypes
import os
import subprocess
import sys
from multiprocessing import current_process
import threading
import time
import uuid
import io
import webbrowser
from concurrent.futures import ThreadPoolExecutor
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from functools import lru_cache
from urllib.parse import urlparse,parse_qs,unquote
from urllib.request import urlopen

from em_solver import ROOT, AIRCRAFT, DEFAULTS, settings, compute, contour_levels as validate_contour_levels
from em_entries import ENTRY_ID
from aircraft_catalog import source,asset_sources
if current_process().name=='MainProcess':


    from em_plot import add_boundary_hover,write_exports,prepare_exports,prepare_export_data,export_csv,export_figure,preview_payload,contour_paths
from vehicle_names import source_paths as name_sources
from release_info import release_info
from em_data import clone

APP=ROOT/'app'; OUTPUT=Path(os.environ.get('WT_EM_OUTPUT_DIR',ROOT/'outputs/em'))
JOBS={}; LOCK=threading.Lock(); WORKER=ThreadPoolExecutor(max_workers=1)
FIGURE_LOCK=threading.Lock()
PREVIEW_WORKER=ThreadPoolExecutor(max_workers=1)
PREVIEWS={}
RESTART_REQUESTED=threading.Event()
NAME_REVISION=hashlib.sha256(b''.join(path.read_bytes() for path in name_sources())+
    (APP/'fonts/wt-symbols.ttf').read_bytes()+b'symbol-export-v1-torque-mode-v1').hexdigest()[:12]
FIGURE_REVISION=hashlib.sha256((ROOT/'scripts/em_plot.py').read_bytes()).hexdigest()[:12]
ALLOWED_ORIGINS={origin.strip().rstrip('/') for origin in os.environ.get('WT_EM_ALLOWED_ORIGINS','').split(',') if origin.strip()}
MAX_PENDING_JOBS=max(1,int(os.environ.get('WT_EM_MAX_PENDING_JOBS','2')))
EPHEMERAL_RESULTS=True
RESULTS={}
MAX_FINISHED_JOBS=2


def named_data(key):
    with LOCK:result=RESULTS.get(key)
    if result is None:raise FileNotFoundError('Result expired')
    return result['data']


def download_data(key):
    """Prepare once on demand when exports were disabled during calculation.

    Chart/point endpoints never enter this path. Keep the preparation under the
    figure lock so simultaneous downloads share one result, without holding the
    job-status lock during conversion. Expired results are never reinserted.
    """
    with FIGURE_LOCK:
        with LOCK:result=RESULTS.get(key)
        if result is None:raise FileNotFoundError('Result expired')
        if not result.get('exports_ready',False):
            data=prepare_export_data(result['data'])
            with LOCK:
                result.update(data=data,exports_ready=True)
        return result['data']


@lru_cache(maxsize=4)
def requested_contours(key,levels):
    data=named_data(key)
    validate_contour_coverage(data,levels)
    return contour_paths(data,levels)


def validate_contour_coverage(data,levels):
    if levels is None:return
    for aircraft in data['aircraft']:
        checked=aircraft.get('interpolation',{})
        if checked.get('scope')=='contours':
            if any(level not in checked.get('checked_sep_levels_mps',[]) for level in levels):
                raise ValueError('Calculate again to validate the newly requested SEP contours')
            continue
        low,high=checked.get('checked_sep_range_mps',[-300.,300.])
        if any(level<low or level>high for level in levels):
            raise ValueError('Requested SEP contour lies outside the checked range; calculate again with these levels')


def query_contour_levels(url):
    params=parse_qs(urlparse(url).query,keep_blank_values=True)
    if 'levels' not in params:return None
    if len(params['levels'])!=1:raise ValueError('Specify one SEP contour list')
    raw=params['levels'][0]
    try:values=[] if not raw else [float(item) for item in raw.split(',')]
    except ValueError as error:raise ValueError('Invalid SEP contour value') from error
    return tuple(validate_contour_levels(values))


@lru_cache(maxsize=4)
def chart_payload(key):
    with LOCK:result=RESULTS.get(key)
    if result is None:raise FileNotFoundError('Result expired')
    return result['chart']


POINT_LOCK=threading.Lock()


@lru_cache(maxsize=4)
def point_solver(key,entry):
    """The full-model solver one chart entry was calculated with (the SEP curve solver for march charts)."""
    from em_solver import TrimSolver,aircraft_settings,settings
    from em_stationary_baseline import BaselineSolver
    data=named_data(key)
    aircraft=next((a for a in data['aircraft'] if a['id']==entry),None)
    if aircraft is None:raise FileNotFoundError('Unknown aircraft entry')
    name=aircraft.get('aircraft_id',aircraft['id'])
    config=data['settings']
    chosen=next((e for e in config.get('entries') or () if e['id']==entry),None)
    condition=aircraft.get('settings') or (settings(dict(config,**dict(chosen['settings'],entries=None,aircraft=[name],
        aircraft_settings={},compare_instructor=False))) if chosen else aircraft_settings(config,name))
    condition=settings({k:v for k,v in condition.items() if k!='_over_rev_limits'})
    if condition['trim_solver_mode']!='stationary':return TrimSolver(name,condition)
    march=aircraft.get('chart_solver')=='march'
    solver=BaselineSolver(name,settings(dict(condition,sideslip_recovery=False)) if march else condition)
    if march:
        import em_march_native
        solver.soft_admission=True
        em_march_native.attach(solver)
    return solver


def pitch_percent(point,solver):
    """Clicked-point report only: the in-game prop pitch control value (%) of each propeller.

    Manual control reports its own setting. Under automatic control it is the manual setting that
    holds the same state, by the game's control law: blade-angle props (governor 6/8)
    pitch = pitch_max - % x (pitch_max - pitch_min); RPM-governor props (1/2)
    target RPM = min + % x (max - min), max = boost RPM while WEP is on. Fixed pitch: none.
    """
    from em_manual_controls import manual_rpm_governor
    propulsion=point.get('propulsion')
    properties=getattr(getattr(solver,'engine',None),'properties',None) if propulsion else None
    propellers=(properties or {}).get('propellers')
    if not propellers or len(propellers)!=len(propulsion.get('propeller_pitch_deg') or ()):return
    controls=propulsion.get('controls') or {}
    automatic,commands=controls.get('automatic') or (),controls.get('commands') or ()
    targets=(propulsion.get('stationarity') or {}).get('shaft_target_rpm') or ()
    shaft={link['index']:t for t,transmission in enumerate(properties.get('transmissions') or ())
           for link in transmission['propellers']}
    rpm=lambda omega:omega*60./(2.*math.pi)
    def percent(i,deg,pp):
        if not pp['governor']:return None
        if i<len(automatic) and not automatic[i] and i<len(commands):return commands[i]/255.*100.
        if manual_rpm_governor(pp):
            t=shaft.get(i)
            target=(targets[t] if t is not None and t<len(targets) and targets[t] is not None else
                    (propulsion.get('engine_rpm') or [None])[min(t or 0,len(propulsion.get('engine_rpm') or [0])-1)])
            low,high=rpm(pp['min_omega']),rpm(pp['boost_omega'] if controls.get('afterburner') else pp['max_omega'])
            return min(100.,max(0.,(target-low)/(high-low)*100.)) if target is not None and high>low else None
        low,high=pp['pitch_min'],pp['pitch_max']
        return min(100.,max(0.,(high-math.radians(deg))/(high-low)*100.)) if high>low else None
    propulsion['propeller_pitch_percent']=[percent(i,deg,q['properties']) for i,(deg,q) in
                                           enumerate(zip(propulsion['propeller_pitch_deg'],propellers))]


def control_percent(point,inverted):
    """Point report only: each surface's deflection (% of full travel) in the pilot's sense, elevator + nose up,
    ailerons + roll right, rudder + nose right. The solver's commands (aileron, elevator, rudder) are + roll right,
    + nose right (body z points left: the rudder's force goes to +z), and + nose up unless the aircraft's elevator
    is inverted (the canard aircraft). Propeller torque confirms it: Merlin aircraft need + aileron and + rudder."""
    commands=point.get('commands')
    if not commands or len(commands)!=3:return point
    aileron,elevator,rudder=commands
    return dict(point,control_deflection_percent=dict(elevator=(-elevator if inverted else elevator)*100.,
                                                      ailerons=aileron*100.,rudder=rudder*100.))


@lru_cache(maxsize=64)
def elevator_inverted(key,entry):
    """InvertElevator of a chart entry's aircraft; None when it cannot be told (no deflections are then reported)."""
    from aircraft_catalog import load
    try:
        aircraft=next((a for a in named_data(key)['aircraft'] if a['id']==entry),None)
        return bool(load(aircraft.get('aircraft_id',aircraft['id']) if aircraft else
                         entry.removesuffix('__instructor_on').removesuffix('__instructor_off'))['InvertElevator'])
    except (OSError,KeyError,ValueError):return None


def solve_point(key,entry,speed_kmh,load_g):
    """Full-model state at a clicked position of a stored chart, seeded from the nearest admitted sample."""
    for label,value,low,high in (('Speed',speed_kmh,50.,3000.),('Load',load_g,1.,64.)):
        if not math.isfinite(value) or not low<=value<=high:raise ValueError(f'{label} out of range')
    with LOCK:result=RESULTS.get(key)
    if result is None:raise FileNotFoundError('Result expired')
    if entry not in result['points']:raise FileNotFoundError('Unknown aircraft entry')
    from em_sampling import continuation_initial
    with POINT_LOCK:
        solver=point_solver(key,entry)
        seeds=[p for p in result['points'][entry] if p.get('local_valid',p.get('valid')) and p.get('solution') is not None]
        initial=None
        if seeds:
            near=min(seeds,key=lambda p:((p['speed_kmh']-speed_kmh)/speed_kmh)**2+((p['load_g']-load_g)/load_g)**2)
            nominal=lambda n:math.degrees(math.acos(1./max(1.,n)))
            attitude=list(near['solution']);attitude[1]+=nominal(load_g)-nominal(near['load_g'])
            initial=continuation_initial(solver,near,attitude)
        try:point=solver.solve(speed_kmh,load_g,initial)
        except ValueError as error:
            # No candidate established: a gap at this state, not an absence proof.
            point=dict(speed_kmh=speed_kmh,load_g=load_g,status='unresolved',valid=False,chart_usable=False,
                       reasons=['stationary search unresolved'],search_unresolved=str(error))
        pitch_percent(point,solver)
        point=control_percent(point,solver.controls['invert_elevator'])
    def public(value):
        if isinstance(value,dict):return {k:public(v) for k,v in value.items() if k not in ('sticks','trim') and not k.startswith('_')}
        if isinstance(value,(list,tuple)):return [public(v) for v in value]
        return value
    from overrev import annotate
    return annotate(dict(public(point),solved_at_request=True))


def fingerprint():


    h=hashlib.sha256(b'em-cache-format-1');pending=['em_solver','em_plot','em_server'];seen=set()
    while pending:
        name=pending.pop()
        if name in seen:continue
        path=ROOT/'scripts'/f'{name}.py'
        if not path.is_file():continue
        seen.add(name)
        for node in ast.walk(ast.parse(path.read_text())):
            if isinstance(node,ast.ImportFrom) and node.module:pending.append(node.module.split('.')[0])
            elif isinstance(node,ast.Import):pending.extend(a.name.split('.')[0] for a in node.names)
    for name in sorted(seen):
        path=ROOT/'scripts'/f'{name}.py';h.update(name.encode());h.update(path.read_bytes())
    h.update((ROOT/'requirements-plotter.txt').read_bytes())
    h.update((ROOT/'scripts/build_em_backend.py').read_bytes())


    from em_backend import signature
    h.update(signature().encode())
    # The march chart's compiled loop: its sources and verification record.
    from em_march_native import native_digest,STAMP
    h.update(native_digest().encode())
    if STAMP.is_file():h.update(STAMP.read_bytes())
    for name in AIRCRAFT:h.update(source(name).read_bytes())
    for path in asset_sources():h.update(path.name.encode());h.update(path.read_bytes())
    return h.hexdigest()


RUNTIME_FINGERPRINT=fingerprint() if current_process().name=='MainProcess' else None


def key_for(config):
    return hashlib.sha256((RUNTIME_FINGERPRINT+json.dumps(config,sort_keys=True)).encode()).hexdigest()[:20]


def discard_old_results():
    with LOCK:
        finished=sorted((job for job in JOBS.values() if job['status'] in ('complete','stopped')),
                        key=lambda job:job['completed_at'])
        old=finished[:-MAX_FINISHED_JOBS]
        for job in old:
            JOBS.pop(job['id'],None)
            PREVIEWS.pop(job['id'],None)
            RESULTS.pop(job['id'],None)
    if old:
        chart_payload.cache_clear();requested_contours.cache_clear();point_solver.cache_clear()


def run_job(key,config,event):
    preview_future=None
    latest_snapshot=None
    def publish(data):
        nonlocal preview_future,latest_snapshot
        # Preserve raw diagnostics even when the preview renderer is busy.
        # Rendering enriches/mutates its input, so it owns a separate copy.
        latest_snapshot=clone(data)
        if event.is_set():return
        if preview_future is not None and not preview_future.done():return
        def render():
            if event.is_set():return
            with FIGURE_LOCK:chart=preview_payload(data)
            with LOCK:
                if event.is_set() or JOBS[key]['status'] not in ('running','queued'):return
                PREVIEWS[key]=chart
                while len(PREVIEWS)>2:PREVIEWS.pop(next(iter(PREVIEWS)))
                JOBS[key]['preview_revision']=JOBS[key].get('preview_revision',0)+1
                JOBS[key].setdefault('first_preview_s',time.monotonic()-started)
        preview_future=PREVIEW_WORKER.submit(render)
    def update(p):
        with LOCK:
            if event.is_set() or JOBS[key].get('cancel') is not event:raise InterruptedError('Calculation cancelled')
            JOBS[key].update(status='running',progress=p)
    try:
        started=time.monotonic()
        reset_calculation()
        update(dict(done=0,total=len(config['aircraft'])*config['speed_samples']*config['load_samples'],phase='Settling engines'))
        data=compute(config,update,event.is_set,preview=publish)
        latest_snapshot=data
        if event.is_set():raise InterruptedError('Calculation cancelled')
        data['equations_fingerprint']=RUNTIME_FINGERPRINT
        with LOCK:JOBS[key].update(status='exporting')
        with FIGURE_LOCK:prepared=prepare_exports(data,started_at=started,
            include_exports=config.get('enable_exports',False))

        with LOCK:
            if event.is_set() or JOBS[key].get('cancel') is not event:raise InterruptedError('Calculation cancelled')
            RESULTS[key]=prepared
            JOBS[key].update(status='complete',elapsed_s=time.monotonic()-started,
                             completed_at=time.monotonic())
        discard_old_results()
    except InterruptedError:
        # This is a partial snapshot, never a completed envelope. Keep preview
        # interpolation/masking and expose raw data exports only.
        data=clone(latest_snapshot) if latest_snapshot is not None else dict(
            settings=clone(config),aircraft=[],speeds_kmh=[],loads_g=[],
            assumptions=[],elapsed_s=0.,plot_max_turn=10.)
        data.update(preview=True,calculation_stopped=True,calculation_status='stopped',
            coverage_complete=False,equations_fingerprint=RUNTIME_FINGERPRINT,
            snapshot_elapsed_s=data.get('elapsed_s',0.),elapsed_s=time.monotonic()-started,
            partial_result_note='Stopped by user. Only available completed columns/snapshots are retained; '
                'in-flight work and unsearched regions remain unresolved, not infeasible.')
        with LOCK:JOBS[key].update(status='exporting',partial_result=True)
        try:
            with FIGURE_LOCK:prepared=prepare_exports(data,started_at=started,
                include_exports=config.get('enable_exports',False))
            with LOCK:
                if JOBS[key].get('cancel') is event:
                    RESULTS[key]=prepared
                    JOBS[key].update(status='stopped',elapsed_s=time.monotonic()-started,
                        completed_at=time.monotonic(),retained_points=sum(len(a['points']) for a in data['aircraft']))
            discard_old_results()
        except Exception as error:
            with LOCK:JOBS[key].update(status='error',error='Partial export failed: '+str(error))
    except Exception as error:
        import traceback
        traceback.print_exc()
        with LOCK:
            if JOBS[key].get('cancel') is event:JOBS[key].update(status='error',error=str(error))
    finally:
        reset_calculation()


def reset_calculation():
    from em_sampling import _AIRCRAFT_CACHE,_COLUMN_CACHE,worker_solver
    # Healthy processes stay warm. process_pool gives each calculation a new
    # epoch; each worker clears its mutable solver history before its first task.
    # Cancelled or failed pools are shut down by process_pool itself.
    _AIRCRAFT_CACHE.clear();_COLUMN_CACHE.clear();worker_solver.cache_clear()


def start_job(config):
    config=settings(config);key=uuid.uuid4().hex[:20]
    from em_uncorrected import require_chart_correction
    require_chart_correction(config)
    with LOCK:
        if RESTART_REQUESTED.is_set():raise ValueError('Calculator is loading updates. Refresh the page before calculating.')
        pending=sum(job.get('status') in ('queued','running','stopping','exporting') for job in JOBS.values())
        if pending>=MAX_PENDING_JOBS:raise ValueError('Calculation queue is full. Try again after the current jobs finish.')
        event=threading.Event();JOBS[key]=dict(id=key,status='queued',settings=config,cancel=event)
        WORKER.submit(run_job,key,config,event)
    return key


class Handler(BaseHTTPRequestHandler):
    def log_message(self,fmt,*args):
        if not args or ' /api/jobs/' not in str(args[0]):super().log_message(fmt,*args)
    def allowed_origin(self):
        origin=self.headers.get('Origin')
        if not origin:return None
        local={f'http://127.0.0.1:{self.server.server_port}',f'http://localhost:{self.server.server_port}'}
        return origin if origin.rstrip('/') in ALLOWED_ORIGINS|local else None
    def end_headers(self):
        origin=self.allowed_origin()
        if origin:
            self.send_header('Access-Control-Allow-Origin',origin)
            self.send_header('Vary','Origin')
        super().end_headers()
    def respond(self,data,status=200):
        content=orjson.dumps(data,option=orjson.OPT_SERIALIZE_NUMPY)
        self.send_response(status);self.send_header('Content-Type','application/json; charset=utf-8')
        self.send_header('Content-Length',str(len(content)));self.send_header('Cache-Control','no-store');self.end_headers()
        self.wfile.write(content)
    def file(self,path):
        if not path.is_file():return self.respond(dict(error='Not found'),404)
        data=path.read_bytes();self.send_response(200)
        mime=mimetypes.guess_type(str(path))[0] or 'application/octet-stream'
        self.send_header('Content-Type',mime);self.send_header('Content-Length',str(len(data)))
        self.send_header('Cache-Control','no-cache');self.send_header('X-Content-Type-Options','nosniff')
        self.end_headers();self.wfile.write(data)
    def download(self,content,name):
        self.send_response(200)
        self.send_header('Content-Type',mimetypes.guess_type(name)[0] or 'application/octet-stream')
        self.send_header('Content-Length',str(len(content)))
        self.send_header('Cache-Control','no-store')
        self.send_header('Content-Disposition',f'attachment; filename="{name}"')
        self.end_headers();self.wfile.write(content)
    def do_GET(self):
        path=urlparse(self.path).path
        if path=='/api/engagement-research/meta':
            from engagement_research_service import metadata
            try:return self.respond(metadata())
            except (ValueError,OSError) as error:return self.respond(dict(error=str(error)),400)
        if path=='/api/missiles/meta':
            from missile_service import metadata
            return self.respond(metadata())
        if path.startswith('/api/missiles/envelopes/'):
            from missile_envelope_service import get as envelope
            try:since=int(parse_qs(urlparse(self.path).query).get('since',['0'])[0])
            except ValueError:since=0
            job=envelope(path.removeprefix('/api/missiles/envelopes/'),since)
            return self.respond(job if job else dict(error='Unknown envelope job'),200 if job else 404)
        if path.startswith('/api/missiles/jobs/'):
            from missile_service import get
            key=path.removeprefix('/api/missiles/jobs/')
            job=get(key)
            return self.respond(job if job else dict(error='Unknown missile job'),200 if job else 404)
        if path=='/api/health':return self.respond(dict(status='ok',release=release_info()))
        if path=='/api/meta':
            # Only a page load opts into edited equations. Running jobs keep
            # their session; Calculate never scans sources or initiates updates.
            if parse_qs(urlparse(self.path).query).get('refresh')==['1'] and fingerprint()!=RUNTIME_FINGERPRINT:
                with LOCK:
                    busy=any(job.get('status') in ('queued','running','stopping','exporting') for job in JOBS.values())
                    missiles=sys.modules.get('missile_service')
                    if missiles is not None:
                        with missiles.LOCK:
                            busy=busy or any(job.get('status') in ('queued','running') for job in missiles.JOBS.values())
                    envelopes=sys.modules.get('missile_envelope_service')
                    if envelopes is not None:busy=busy or envelopes.busy()
                    if not busy:RESTART_REQUESTED.set()
                if not busy:
                    self.respond(dict(error='Loading updated calculator.',code='server_restarting'),503)
                    threading.Thread(target=self.server.shutdown,daemon=True).start()
                    return
            from engine_performance import without_wep
            return self.respond(dict(aircraft=dict(AIRCRAFT),defaults=DEFAULTS,latest=None,no_wep=without_wep(),
                result_policy=dict(storage='memory',fresh_calculations=True,automatic_saving=False),release=release_info()))
        if path.startswith('/api/loadouts/'):
            name=unquote(path.removeprefix('/api/loadouts/'))
            if name not in AIRCRAFT:return self.respond(dict(error='Unknown aircraft'),404)
            vehicle=parse_qs(urlparse(self.path).query).get('vehicle',[''])[0]
            if len(vehicle)>120:return self.respond(dict(error='Invalid vehicle'),400)
            from aircraft_loadouts import summary as loadout_summary
            try:return self.respond(loadout_summary(name,vehicle))
            except (OSError,ValueError,KeyError) as error:return self.respond(dict(supported=False,reason=str(error)))
        if path.startswith('/api/engine/'):
            # Engine performance page: one aircraft's sweeps. ?sweep=altitude|speed|both&altitude_m=&speed_kmh=&throttle=percent
            # &pitch=auto|percent&level=0|1 (1: SEP and acceleration in 1 g level flight)
            name=unquote(path.removeprefix('/api/engine/'))
            if name not in AIRCRAFT:return self.respond(dict(error='Unknown aircraft'),404)
            query=parse_qs(urlparse(self.path).query)
            try:
                if any(len(v)!=1 for v in query.values()):raise ValueError('Repeated parameter')
                altitude,speed=float(query.get('altitude_m',['0'])[0]),float(query.get('speed_kmh',['300'])[0])
                throttle=float(query.get('throttle',['110'])[0])
                raw=query.get('pitch',['auto'])[0];pitch=None if raw=='auto' else float(raw)
                sweep=query.get('sweep',['both'])[0]
                if sweep not in ('altitude','speed','both'):raise ValueError('Unknown sweep')
                level=query.get('level',['0'])[0]
                if level not in ('0','1'):raise ValueError('level must be 0 or 1')
                for label,value,low,high in (('Altitude',altitude,0.,18000.),('Speed',speed,5.,1600.),('Throttle',throttle,0.,110.),('Propeller pitch',pitch,0.,100.)):
                    if value is not None and not (math.isfinite(value) and low<=value<=high):raise ValueError(f'{label} out of range')
                from engine_performance import compute as engine_performance
                return self.respond(engine_performance(name,altitude,speed,throttle,pitch,pitch_percent,None if sweep=='both' else sweep,level=='1'))
            except ValueError as error:return self.respond(dict(error=str(error)),400)
        if path.startswith('/api/jobs/'):
            parts=path.split('/');key=parts[3]
            if len(key)!=20 or any(c not in '0123456789abcdef' for c in key):return self.respond(dict(error='Invalid job'),400)
            if len(parts)==7 and parts[4]=='point':
                aircraft,index=parts[5:]
                base=aircraft.removesuffix('__instructor_on').removesuffix('__instructor_off')
                legacy=base in AIRCRAFT and aircraft in (base,base+'__instructor_on',base+'__instructor_off')
                if not (legacy or ENTRY_ID.fullmatch(aircraft)) or not index.isdecimal():return self.respond(dict(error='Invalid point'),400)
                try:
                    with LOCK:result=RESULTS.get(key)
                    if result is None:raise FileNotFoundError('Result expired')
                    point=result['points'][aircraft][int(index)];inverted=elevator_inverted(key,aircraft) if point.get('commands') else None
                    return self.respond(point if inverted is None else control_percent(point,inverted))
                except (OSError,IndexError,ValueError,KeyError):return self.respond(dict(error='Point not found'),404)
            if len(parts)==6 and parts[4]=='solve':
                # The full-model state at a clicked chart position: ?speed_kmh=&load_g=
                entry=parts[5]
                base=entry.removesuffix('__instructor_on').removesuffix('__instructor_off')
                if not (ENTRY_ID.fullmatch(entry) or (base in AIRCRAFT and entry in (base,base+'__instructor_on',base+'__instructor_off'))):
                    return self.respond(dict(error='Invalid aircraft entry'),400)
                query=parse_qs(urlparse(self.path).query)
                try:
                    if len(query.get('speed_kmh',()))!=1 or len(query.get('load_g',()))!=1:raise ValueError('speed_kmh and load_g required')
                    return self.respond(solve_point(key,entry,float(query['speed_kmh'][0]),float(query['load_g'][0])))
                except FileNotFoundError as error:return self.respond(dict(error=str(error)),404)
                except ValueError as error:return self.respond(dict(error=str(error)),400)
                except Exception as error:
                    import traceback
                    traceback.print_exc()
                    return self.respond(dict(error='Point solve failed: '+type(error).__name__),500)
            if len(parts)==5 and parts[4]=='preview.json':
                with LOCK:chart=PREVIEWS.get(key)
                return self.respond(chart) if chart else self.respond(dict(error='Preview not ready'),404)
            if len(parts)==5 and parts[4]=='contours.json':
                try:
                    levels=query_contour_levels(self.path)
                    if levels is None:raise ValueError('SEP contour levels required')
                    return self.respond(dict(contours=requested_contours(key,levels)))
                except ValueError as error:return self.respond(dict(error=str(error)),400)
                except OSError:return self.respond(dict(error='Not found'),404)
            if len(parts)==5 and parts[4] in ('data.json','chart.json','samples.csv','diagram.png','diagram.svg','diagram.pdf'):
                try:
                    name=parts[4];data=named_data(key)
                    if data.get('calculation_stopped') and name.startswith('diagram.'):
                        return self.respond(dict(error='Stopped results support JSON and CSV exports; diagram refinement is incomplete.'),400)
                    if name=='chart.json':return self.respond(chart_payload(key))
                    data=download_data(key)
                    download_name=('partial-' if data.get('calculation_stopped') else '')+name
                    if name=='data.json':return self.download(orjson.dumps(data,option=orjson.OPT_SERIALIZE_NUMPY),download_name)
                    if name=='samples.csv':return self.download(export_csv(data).encode(),download_name)
                    levels=query_contour_levels(self.path);validate_contour_coverage(data,levels)
                    buffer=io.BytesIO()
                    with FIGURE_LOCK:export_figure(data,buffer,levels=levels,format=name.split('.')[-1])
                    return self.download(buffer.getvalue(),name)
                except FileNotFoundError:return self.respond(dict(error='Not found'),404)
                except ValueError as error:return self.respond(dict(error=str(error)),400)
            with LOCK:job={k:v for k,v in JOBS.get(key,{}).items() if k!='cancel'}
            return self.respond(job if job else dict(error='Unknown job'),200 if job else 404)
        if path=='/favicon.ico':self.send_response(204);self.end_headers();return
        names={'/':'index.html','/index.html':'index.html','/app.js':'app.js','/config.js':'config.js','/styles.css':'styles.css','/vendor/plotly.min.js':'vendor/plotly.min.js'}
        names.update({'/release.json':'release.json','/release-links.js':'release-links.js','/release-links.css':'release-links.css'})
        names.update({'/missiles.js':'missiles.js','/missiles.css':'missiles.css'})
        names.update({'/missile-envelope.js':'missile-envelope.js','/missile-envelope.css':'missile-envelope.css'})
        names.update({'/engine.js':'engine.js','/engine.css':'engine.css'})
        names.update({f'/engagement-research.{extension}':f'engagement-research.{extension}' for extension in ('html','css','js')})
        names['/fonts/wt-symbols.ttf']='fonts/wt-symbols.ttf'
        names.update({f'/icons/neothunderism-{size}.png':f'icons/neothunderism-{size}.png' for size in (16,32,180,192,512)})
        names.update({f'/icons/neothunderism-{asset}':f'icons/neothunderism-{asset}' for asset in ('mark.svg','mark.png','app.svg','favicon.svg')})
        if path in names:return self.file(APP/names[path])
        return self.respond(dict(error='Not found'),404)
    def do_POST(self):
        path=urlparse(self.path).path
        origin=self.headers.get('Origin')
        if origin and not self.allowed_origin():
            return self.respond(dict(error='Local or configured public origin required'),403)
        try:
            length=int(self.headers.get('Content-Length','0'))
            if length<0 or length>32768:raise ValueError('Request too large')
            body=json.loads(self.rfile.read(length) or b'{}')
            if not isinstance(body,dict):raise ValueError('Expected a JSON object')
            if path=='/api/engagement-research/run':
                from engagement_research_service import run
                return self.respond(run(body))
            if path=='/api/missiles/jobs':
                from missile_service import start
                with LOCK:
                    if RESTART_REQUESTED.is_set():raise ValueError('Calculator is loading updates. Refresh the page before calculating.')
                    key=start(body)
                return self.respond(dict(id=key),202)
            if path.startswith('/api/missiles/jobs/') and path.endswith('/cancel'):
                from missile_service import cancel
                return self.respond(dict(cancelled=cancel(path.split('/')[4])))
            if path=='/api/missiles/envelopes':
                from missile_envelope_service import start as start_envelope
                with LOCK:
                    if RESTART_REQUESTED.is_set():raise ValueError('Calculator is loading updates. Refresh the page before calculating.')
                    key=start_envelope(body)
                return self.respond(dict(id=key),202)
            if path.startswith('/api/missiles/envelopes/') and path.endswith('/cancel'):
                from missile_envelope_service import cancel as cancel_envelope
                return self.respond(dict(cancelled=cancel_envelope(path.split('/')[4])))
            if path=='/api/jobs':
                key=start_job(body);return self.respond(dict(id=key),202)
            if path=='/api/trial':
                from em_uncorrected import request
                return self.respond(request(body))
            if path.startswith('/api/jobs/') and path.endswith('/cancel'):
                key=path.split('/')[3]
                with LOCK:
                    job=JOBS.get(key)
                    if job and 'cancel' in job and job['status'] in ('queued','running','stopping','exporting'):
                        job['cancel'].set()
                        job['status']='stopping'
                return self.respond(dict(cancelled=bool(job)))
            return self.respond(dict(error='Not found'),404)
        except (ValueError,TypeError,KeyError) as error:return self.respond(dict(error=str(error)),400)
    def do_OPTIONS(self):
        if not self.allowed_origin():return self.respond(dict(error='Origin not allowed'),403)
        self.send_response(204)
        self.send_header('Access-Control-Allow-Methods','GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers','Content-Type')
        self.send_header('Access-Control-Max-Age','86400')
        self.end_headers()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host',default=os.environ.get('WT_EM_HOST','127.0.0.1'))
    parser.add_argument('--port',type=int,default=int(os.environ.get('PORT','8765')))
    parser.add_argument('--open',action='store_true');parser.add_argument('--sample',action='store_true')
    args=parser.parse_args()
    if args.sample:
        config=settings();key=key_for(config);event=threading.Event()
        JOBS[key]=dict(id=key,status='queued',settings=config,cancel=event);run_job(key,config,event)
        print(json.dumps({k:v for k,v in JOBS[key].items() if k!='cancel'}),flush=True)
        if JOBS[key]['status']!='complete':raise SystemExit(1)
        return
    url=f'http://127.0.0.1:{args.port}'
    try:server=ThreadingHTTPServer((args.host,args.port),Handler)
    except OSError as error:
        if args.open and error.errno==errno.EADDRINUSE:
            try:
                with urlopen(url+'/api/meta',timeout=2) as response:meta=json.load(response)
                if set(meta['aircraft'])==set(AIRCRAFT) and 'defaults' in meta and meta.get('release',{}).get('version')==release_info()['version']:
                    print('Opening running EM plotter: '+url,flush=True);webbrowser.open(url);return
                raise RuntimeError('Another calculator version is using port '+str(args.port)+'. Close its window before launching this version.')
            except (OSError,ValueError,KeyError,TypeError):pass
        raise
    print('EM plotter: '+url,flush=True)
    if args.open:threading.Timer(.4,lambda:webbrowser.open(url)).start()
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:
        for job in list(JOBS.values()):
            if 'cancel' in job:job['cancel'].set()
        server.server_close();WORKER.shutdown(wait=True,cancel_futures=True)
        PREVIEW_WORKER.shutdown(wait=True,cancel_futures=True)
        from em_workers import shutdown
        shutdown()
    if RESTART_REQUESTED.is_set():
        from em_session import cleanup,fresh_environment
        command=[sys.executable,*[arg for arg in sys.argv if arg!='--open']]
        env=fresh_environment()
        print('Equation files changed; restarting EM plotter.',flush=True)
        if os.name=='nt':
            subprocess.Popen(command,cwd=ROOT,env=env,creationflags=subprocess.CREATE_NEW_PROCESS_GROUP)
            cleanup()
            os._exit(0)
        else:
            cleanup()
            os.execve(sys.executable,command,env)


if __name__=='__main__':main()
