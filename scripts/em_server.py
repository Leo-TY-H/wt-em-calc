"""Interactive EM plotter API and local UI. Run with --open for local use."""
import argparse
import ast
import errno
import hashlib
import json
import orjson
import mimetypes
import os
from multiprocessing import current_process
import threading
import time
import uuid
import webbrowser
from concurrent.futures import ThreadPoolExecutor
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from functools import lru_cache
from urllib.parse import urlparse
from urllib.request import urlopen

import numpy as np
from scipy.interpolate import PchipInterpolator

from em_solver import ROOT, AIRCRAFT, DEFAULTS, settings, compute
from em_entries import ENTRY_ID
from em_sampling import column_curve
from aircraft_catalog import source,asset_sources
if current_process().name=='MainProcess':
    # Windows spawn imports this entry module in every numerical worker.
    # Plotting belongs to the server; loading Matplotlib in twelve workers
    # delays the first numerical results and duplicates its font/cache state.
    from em_plot import add_boundary_hover,write_exports,export_figure,preview_payload
from vehicle_names import refresh_result_names, SOURCE as NAME_SOURCE

APP=ROOT/'app'; OUTPUT=Path(os.environ.get('WT_EM_OUTPUT_DIR',ROOT/'outputs/em')); OUTPUT.mkdir(parents=True,exist_ok=True)
JOBS={}; LOCK=threading.Lock(); WORKER=ThreadPoolExecutor(max_workers=1)
FIGURE_LOCK=threading.Lock()
PREVIEW_WORKER=ThreadPoolExecutor(max_workers=1)
PREVIEWS={}
NAME_REVISION=hashlib.sha256(NAME_SOURCE.read_bytes()+
    (APP/'fonts/wt-symbols.ttf').read_bytes()+b'symbol-export-v1-torque-mode-v1').hexdigest()[:12]
ALLOWED_ORIGINS={origin.strip().rstrip('/') for origin in os.environ.get('WT_EM_ALLOWED_ORIGINS','').split(',') if origin.strip()}
MAX_PENDING_JOBS=max(1,int(os.environ.get('WT_EM_MAX_PENDING_JOBS','2')))


def named_data(key):
    return refresh_result_names(orjson.loads((OUTPUT/key/'data.json').read_bytes()),AIRCRAFT)


def named_export(key,name):
    """Keep historical numerical files intact while refreshing download labels."""
    path=OUTPUT/key/(NAME_REVISION+'-'+name)
    with FIGURE_LOCK:
        if not path.exists():
            data=named_data(key)
            if name=='data.json':content=orjson.dumps(data)
            else:
                from em_plot import export_csv
                content=export_csv(data).encode()
            temporary=path.with_suffix(path.suffix+'.tmp')
            temporary.write_bytes(content);temporary.replace(path)
    return path


def figure_export(key,name):
    """Render a requested download without delaying the interactive chart."""
    path=OUTPUT/key/(NAME_REVISION+'-'+name)
    if path.exists():return path
    # Matplotlib is process-global. Serialize export requests, and expose each
    # completed file atomically so concurrent downloads never see a partial PDF.
    with FIGURE_LOCK:
        if not path.exists():
            data=named_data(key)
            temporary=path.with_name(path.stem+'.tmp'+path.suffix)
            export_figure(data,temporary);temporary.replace(path)
    return path


@lru_cache(maxsize=16)
def point_offsets(key,aircraft):
    return json.loads((OUTPUT/key/(aircraft+'-offsets.json')).read_text())


@lru_cache(maxsize=4)
def chart_payload(key):
    chart=orjson.loads((OUTPUT/key/'chart.json').read_bytes())
    ready=orjson.loads((OUTPUT/key/'ready.json').read_bytes())
    if 'total_elapsed_s' in ready:
        chart['calculation_elapsed_s']=chart['elapsed_s']
        chart['elapsed_s']=ready['total_elapsed_s']
    refresh_result_names(chart,AIRCRAFT)
    if chart.get('boundary_hover_ready'):return chart
    data=orjson.loads((OUTPUT/key/'data.json').read_bytes())
    return add_boundary_hover(chart,data)


def fingerprint():
    # Other investigations share this workspace. Only the transitive local
    # equation/plot dependencies may invalidate these numerical results.
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
    # Generated kernel helpers are read by the builder, rather than imported
    # by the runtime equation graph. Include their complete build identity so
    # an implementation change cannot reuse a plot from an older backend.
    from em_backend import signature
    h.update(signature().encode())
    for name in AIRCRAFT:h.update(source(name).read_bytes())
    for path in asset_sources():h.update(path.name.encode());h.update(path.read_bytes())
    return h.hexdigest()


RUNTIME_FINGERPRINT=fingerprint() if current_process().name=='MainProcess' else None


def key_for(config):
    return hashlib.sha256((RUNTIME_FINGERPRINT+json.dumps(config,sort_keys=True)).encode()).hexdigest()[:20]


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
        update(dict(done=0,total=len(config['aircraft'])*config['speed_samples']*config['load_samples'],phase='Settling engines'))
        data=compute(config,update,event.is_set,preview=publish)
        if event.is_set():raise InterruptedError('Calculation cancelled')
        data['equations_fingerprint']=RUNTIME_FINGERPRINT
        with LOCK:JOBS[key].update(status='exporting')
        with FIGURE_LOCK:write_exports(data,OUTPUT/key,figures=False,started_at=started)
        # Only complete artifacts become the default shown on next launch.
        with LOCK:
            if event.is_set() or JOBS[key].get('cancel') is not event:raise InterruptedError('Calculation cancelled')
            (OUTPUT/'latest.json').write_text(json.dumps(dict(id=key,settings=config)))
            JOBS[key].update(status='complete',elapsed_s=time.monotonic()-started)
    except InterruptedError:
        with LOCK:
            if JOBS[key].get('cancel') is event:JOBS[key].update(status='cancelled')
    except Exception as error:
        import traceback
        traceback.print_exc()
        with LOCK:
            if JOBS[key].get('cancel') is event:JOBS[key].update(status='error',error=str(error))


def start_job(config):
    if fingerprint()!=RUNTIME_FINGERPRINT:
        raise ValueError('Equation files changed. Restart the plotter before calculating with the new equations.')
    config=settings(config);key=key_for(config)
    with LOCK:
        if key in JOBS and JOBS[key]['status'] in ('queued','running','exporting','complete'):return key
        if (OUTPUT/key/'ready.json').exists() and (OUTPUT/key/'chart.json').exists() and (OUTPUT/key/'data.json').exists():
            JOBS[key]=dict(id=key,status='complete',cached=True,settings=config)
        else:
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
    def do_GET(self):
        path=urlparse(self.path).path
        if path=='/api/health':return self.respond(dict(status='ok'))
        if path=='/api/meta':
            latest=None
            try:
                item=json.loads((OUTPUT/'latest.json').read_text())
                if key_for(item['settings'])==item['id'] and (OUTPUT/item['id']/'data.json').exists():latest=item['id']
            except (OSError,ValueError,KeyError):pass
            return self.respond(dict(aircraft=dict(AIRCRAFT),defaults=DEFAULTS,latest=latest))
        if path.startswith('/api/jobs/'):
            parts=path.split('/');key=parts[3]
            if len(key)!=20 or any(c not in '0123456789abcdef' for c in key):return self.respond(dict(error='Invalid job'),400)
            if len(parts)==7 and parts[4]=='point':
                aircraft,index=parts[5:]
                base=aircraft.removesuffix('__instructor_on').removesuffix('__instructor_off')
                legacy=base in AIRCRAFT and aircraft in (base,base+'__instructor_on',base+'__instructor_off')
                if not (legacy or ENTRY_ID.fullmatch(aircraft)) or not index.isdecimal():return self.respond(dict(error='Invalid point'),400)
                try:
                    offset=point_offsets(key,aircraft)[int(index)]
                    with (OUTPUT/key/(aircraft+'-points.jsonl')).open('rb') as record:
                        record.seek(offset);point=json.loads(record.readline())
                    return self.respond(point)
                except (OSError,IndexError,ValueError):return self.respond(dict(error='Point not found'),404)
            if len(parts)==5 and parts[4]=='preview.json':
                with LOCK:chart=PREVIEWS.get(key)
                return self.respond(chart) if chart else self.respond(dict(error='Preview not ready'),404)
            if len(parts)==5 and parts[4] in ('data.json','chart.json','samples.csv','diagram.png','diagram.svg','diagram.pdf'):
                if parts[4]=='chart.json':
                    try:return self.respond(chart_payload(key))
                    except OSError:return self.respond(dict(error='Not found'),404)
                if parts[4].startswith('diagram.'):
                    try:return self.file(figure_export(key,parts[4]))
                    except FileNotFoundError:return self.respond(dict(error='Not found'),404)
                try:return self.file(named_export(key,parts[4]))
                except FileNotFoundError:return self.respond(dict(error='Not found'),404)
            with LOCK:job={k:v for k,v in JOBS.get(key,{}).items() if k!='cancel'}
            if not job and (OUTPUT/key/'data.json').exists():job=dict(id=key,status='complete',cached=True)
            return self.respond(job if job else dict(error='Unknown job'),200 if job else 404)
        if path=='/favicon.ico':self.send_response(204);self.end_headers();return
        names={'/':'index.html','/index.html':'index.html','/app.js':'app.js','/config.js':'config.js','/styles.css':'styles.css','/vendor/plotly.min.js':'vendor/plotly.min.js'}
        names['/fonts/wt-symbols.ttf']='fonts/wt-symbols.ttf'
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
                if set(meta['aircraft'])==set(AIRCRAFT) and 'defaults' in meta:
                    print('Opening running EM plotter: '+url,flush=True);webbrowser.open(url);return
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


if __name__=='__main__':main()
