import argparse
import ast
import errno
import hashlib
import json
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
from urllib.parse import urlparse,parse_qs
from urllib.request import urlopen

import numpy as np
from scipy.interpolate import PchipInterpolator

from em_solver import ROOT, AIRCRAFT, DEFAULTS, settings, compute, contour_levels as validate_contour_levels
from em_entries import ENTRY_ID
from em_sampling import column_curve
from aircraft_catalog import source,asset_sources
if current_process().name=='MainProcess':


    from em_plot import add_boundary_hover,write_exports,prepare_exports,export_csv,export_figure,preview_payload,contour_paths
from vehicle_names import refresh_result_names, SOURCE as NAME_SOURCE
from release_info import release_info

APP=ROOT/'app'; OUTPUT=Path(os.environ.get('WT_EM_OUTPUT_DIR',ROOT/'outputs/em'))
JOBS={}; LOCK=threading.Lock(); WORKER=ThreadPoolExecutor(max_workers=1)
FIGURE_LOCK=threading.Lock()
PREVIEW_WORKER=ThreadPoolExecutor(max_workers=1)
PREVIEWS={}
RESTART_REQUESTED=threading.Event()
NAME_REVISION=hashlib.sha256(NAME_SOURCE.read_bytes()+
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


def fingerprint():


    h=hashlib.sha256(b'em-cache-format-1');pending=['em_solver','em_plot'];seen=set()
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
    for name in AIRCRAFT:h.update(source(name).read_bytes())
    for path in asset_sources():h.update(path.name.encode());h.update(path.read_bytes())
    return h.hexdigest()


RUNTIME_FINGERPRINT=fingerprint() if current_process().name=='MainProcess' else None


class RestartRequired(Exception):
    pass


def key_for(config):
    return hashlib.sha256((RUNTIME_FINGERPRINT+json.dumps(config,sort_keys=True)).encode()).hexdigest()[:20]


def discard_old_results():
    with LOCK:
        finished=sorted((job for job in JOBS.values() if job['status']=='complete'),
                        key=lambda job:job['completed_at'])
        old=finished[:-MAX_FINISHED_JOBS]
        for job in old:
            JOBS.pop(job['id'],None)
            PREVIEWS.pop(job['id'],None)
            RESULTS.pop(job['id'],None)
    if old:
        chart_payload.cache_clear();requested_contours.cache_clear()


def run_job(key,config,event):
    preview_future=None
    def publish(data):
        nonlocal preview_future
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
        if event.is_set():raise InterruptedError('Calculation cancelled')
        data['equations_fingerprint']=RUNTIME_FINGERPRINT
        with LOCK:JOBS[key].update(status='exporting')
        with FIGURE_LOCK:prepared=prepare_exports(data,started_at=started)

        with LOCK:
            if event.is_set() or JOBS[key].get('cancel') is not event:raise InterruptedError('Calculation cancelled')
            RESULTS[key]=prepared
            JOBS[key].update(status='complete',elapsed_s=time.monotonic()-started,
                             completed_at=time.monotonic())
        discard_old_results()
    except InterruptedError:
        with LOCK:
            if JOBS[key].get('cancel') is event:JOBS[key].update(status='cancelled')
    except Exception as error:
        import traceback
        traceback.print_exc()
        with LOCK:
            if JOBS[key].get('cancel') is event:JOBS[key].update(status='error',error=str(error))
    finally:
        reset_calculation()


def reset_calculation():
    from em_workers import shutdown
    from em_sampling import _AIRCRAFT_CACHE,_COLUMN_CACHE,worker_solver
    shutdown()
    _AIRCRAFT_CACHE.clear();_COLUMN_CACHE.clear();worker_solver.cache_clear()


def start_job(config):
    if fingerprint()!=RUNTIME_FINGERPRINT:
        raise RestartRequired('Equation files changed; restarting the calculator with the new equations.')
    config=settings(config);key=uuid.uuid4().hex[:20]
    with LOCK:
        pending=sum(job.get('status') in ('queued','running','exporting') for job in JOBS.values())
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
        if path=='/api/missiles/meta':
            from missile_service import metadata
            return self.respond(metadata())
        if path.startswith('/api/missiles/jobs/'):
            from missile_service import get
            key=path.removeprefix('/api/missiles/jobs/')
            job=get(key)
            return self.respond(job if job else dict(error='Unknown missile job'),200 if job else 404)
        if path=='/api/health':return self.respond(dict(status='ok',release=release_info()))
        if path=='/api/meta':
            return self.respond(dict(aircraft=dict(AIRCRAFT),defaults=DEFAULTS,latest=None,
                result_policy=dict(storage='memory',fresh_calculations=True,automatic_saving=False),release=release_info()))
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
                    return self.respond(result['points'][aircraft][int(index)])
                except (OSError,IndexError,ValueError,KeyError):return self.respond(dict(error='Point not found'),404)
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
                    if name=='chart.json':return self.respond(chart_payload(key))
                    if name=='data.json':return self.download(orjson.dumps(data,option=orjson.OPT_SERIALIZE_NUMPY),name)
                    if name=='samples.csv':return self.download(export_csv(data).encode(),name)
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
        names['/workbench.css']='workbench.css'
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
            if path=='/api/missiles/jobs':
                from missile_service import start
                return self.respond(dict(id=start(body)),202)
            if path.startswith('/api/missiles/jobs/') and path.endswith('/cancel'):
                from missile_service import cancel
                return self.respond(dict(cancelled=cancel(path.split('/')[4])))
            if path=='/api/jobs':
                key=start_job(body);return self.respond(dict(id=key),202)
            if path.startswith('/api/jobs/') and path.endswith('/cancel'):
                key=path.split('/')[3]
                with LOCK:
                    job=JOBS.get(key)
                    if job and 'cancel' in job:
                        job['cancel'].set()
                        if job['status']=='queued':job['status']='cancelled'
                return self.respond(dict(cancelled=bool(job)))
            return self.respond(dict(error='Not found'),404)
        except RestartRequired as error:
            self.respond(dict(error=str(error),code='server_restarting'),503)
            if not RESTART_REQUESTED.is_set():
                RESTART_REQUESTED.set()
                threading.Thread(target=self.server.shutdown,daemon=True).start()
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
        server.server_close();WORKER.shutdown(wait=False,cancel_futures=True)
    if RESTART_REQUESTED.is_set():
        command=[sys.executable,*sys.argv]
        print('Equation files changed; restarting EM plotter.',flush=True)
        if os.name=='nt':
            subprocess.Popen(command,cwd=ROOT,creationflags=subprocess.CREATE_NEW_PROCESS_GROUP)
            os._exit(0)
        else:
            os.execv(sys.executable,command)


if __name__=='__main__':main()
