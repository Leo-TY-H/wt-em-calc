"""Isolated altitude-SEP app; leaves the EM process, jobs and caches untouched."""
import altitude_runtime
import argparse
import ast
import errno
import hashlib
import json
import mimetypes
import os
from pathlib import Path
import re
import threading
import time
import traceback
import webbrowser
from concurrent.futures import ThreadPoolExecutor
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse,parse_qs
from urllib.request import urlopen

import orjson
from altitude_envelope import AIRCRAFT, DEFAULTS, settings, compute
from altitude_plot import contours, chart_payload, export_csv, export_figure
from altitude_energy import energy_guide
from aircraft_catalog import asset_sources, source

ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / 'app'
OUTPUT = Path(os.environ.get('WT_ALTITUDE_OUTPUT_DIR', ROOT / 'outputs/altitude'))
JOBS = {}
LOCK = threading.Lock()
FIGURE_LOCK = threading.Lock()
WORKER = ThreadPoolExecutor(max_workers=1)
JOB_ID = re.compile(r'[a-f0-9]{20}\Z')


def fingerprint():
    """Hash this feature and its equation/data dependencies, without EM plots."""
    from em_backend import MODULES
    pending = ['altitude_envelope', 'altitude_plot', 'altitude_runtime', 'altitude_energy', 'build_em_backend', *MODULES]
    seen = set(); digest = hashlib.sha256(b'altitude-sep-v1')
    while pending:
        name = pending.pop()
        path = ROOT / 'scripts' / (name + '.py')
        if name in seen or not path.is_file():
            continue
        seen.add(name)
        content = path.read_bytes(); digest.update(name.encode()); digest.update(content); INPUT_PATHS.add(path)
        for node in ast.walk(ast.parse(content)):
            if isinstance(node, ast.ImportFrom) and node.module:
                pending.append(node.module.split('.')[0])
            elif isinstance(node, ast.Import):
                pending.extend(a.name.split('.')[0] for a in node.names)
    for name in sorted(AIRCRAFT):
        digest.update(source(name).read_bytes()); INPUT_PATHS.add(source(name))
    for path in asset_sources():
        digest.update(path.name.encode()); digest.update(path.read_bytes()); INPUT_PATHS.add(path)
    return digest.hexdigest()


REVISION = None
POOL = POOL_STOP = None
INPUT_PATHS = set()
INPUT_SIGNATURE = None


def input_signature():
    return tuple((str(p),p.stat().st_mtime_ns,p.stat().st_size) for p in sorted(INPUT_PATHS))


def inputs_changed():
    return input_signature()!=INPUT_SIGNATURE


def dump(value):
    return orjson.dumps(value, option=orjson.OPT_SERIALIZE_NUMPY)


def run_job(key, config, event):
    started = time.monotonic()
    def update(progress):
        with LOCK:JOBS[key].update(status='running', progress=progress)
    try:
        if event.is_set():return
        POOL_STOP.clear()
        update(dict(samples=0,done=0,total=1,phase='Solving level flight'))
        data=compute(config,update,event.is_set,pool=POOL,stop=POOL_STOP)
        if event.is_set():raise InterruptedError('Calculation cancelled')
        with LOCK:JOBS[key]['status']='exporting'
        with FIGURE_LOCK:data['contours']=contours(data)
        if inputs_changed():raise ValueError('Aircraft data changed. Restart the altitude plotter.')
        data['equations_fingerprint']=REVISION
        directory=OUTPUT/key;directory.mkdir(parents=True,exist_ok=True)
        data['elapsed_s']=time.monotonic()-started
        artifacts={'data.json':dump(data),'chart.json':dump(chart_payload(data)),
                   'samples.csv':export_csv(data).encode()}
        for name,content in artifacts.items():
            temporary=directory/(name+'.tmp');temporary.write_bytes(content);temporary.replace(directory/name)
        with LOCK:
            if event.is_set():raise InterruptedError('Calculation cancelled')
            elapsed=time.monotonic()-started
            (directory/'ready.json').write_bytes(dump(dict(elapsed_s=elapsed)))
            JOBS[key].update(status='complete',elapsed_s=elapsed)
    except InterruptedError:
        with LOCK:JOBS[key]['status']='cancelled'
    except Exception as error:
        traceback.print_exc()
        with LOCK:JOBS[key].update(status='error',error=str(error))


def start_job(values):
    config = settings(values)
    if inputs_changed():
        raise ValueError('Flight-model files changed. Restart the altitude plotter to load the updated equations.')
    key = hashlib.sha256((REVISION + json.dumps(config, sort_keys=True)).encode()).hexdigest()[:20]
    with LOCK:
        if key in JOBS and JOBS[key]['status'] in ('running', 'queued', 'exporting', 'complete'):
            return key
        if all((OUTPUT / key / name).is_file() for name in ('ready.json', 'data.json', 'chart.json', 'samples.csv')):
            JOBS[key] = dict(id=key, status='complete', cached=True, settings=config)
        else:
            if sum(j['status'] in ('queued', 'running', 'exporting') for j in JOBS.values()) >= 2:
                raise ValueError('Altitude calculation queue is full. Wait for a calculation to finish.')
            event = threading.Event()
            JOBS[key] = dict(id=key, status='queued', settings=config, cancel=event)
            WORKER.submit(run_job, key, config, event)
    return key


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        if not args or '/api/altitude/jobs/' not in str(args[0]):
            super().log_message(fmt, *args)

    def respond(self, value, status=200):
        self.send_bytes(dump(value), 'application/json; charset=utf-8', status)

    def send_bytes(self, content, mime, status=200):
        self.send_response(status)
        self.send_header('Content-Type', mime)
        self.send_header('Content-Length', str(len(content)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        try:
            self.end_headers()
            self.wfile.write(content)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def file(self, path):
        if not path.is_file():
            return self.respond(dict(error='Not found'), 404)
        return self.send_bytes(path.read_bytes(), mimetypes.guess_type(str(path))[0] or 'application/octet-stream')

    def do_GET(self):
        path = urlparse(self.path).path
        if path == '/api/altitude/meta':
            return self.respond(dict(aircraft=dict(AIRCRAFT), defaults=DEFAULTS, feature='altitude-sep-v1'))
        if path == '/api/health':
            return self.respond(dict(status='ok', feature='altitude-sep-v1'))
        files = {'/': 'altitude.html', '/altitude.html': 'altitude.html', '/altitude.js': 'altitude.js',
                 '/altitude_surface.js': 'altitude_surface.js',
                 '/altitude.css': 'altitude.css', '/vendor/plotly.min.js': 'vendor/plotly.min.js',
                 '/fonts/wt-symbols.ttf': 'fonts/wt-symbols.ttf'}
        if path in files:
            return self.file(APP / files[path])
        parts = path.strip('/').split('/')
        if len(parts) not in (4, 5, 6) or parts[:3] != ['api', 'altitude', 'jobs'] or not JOB_ID.fullmatch(parts[3]):
            return self.respond(dict(error='Not found'), 404)
        key = parts[3]
        directory = OUTPUT / key
        if len(parts) == 4:
            with LOCK:
                job = {k: v for k, v in JOBS.get(key, {}).items() if k != 'cancel'}
            if not job and (directory / 'ready.json').exists():
                job = dict(id=key, status='complete', cached=True)
            return self.respond(job or dict(error='Unknown job'), 200 if job else 404)
        if not (directory / 'ready.json').is_file():
            return self.respond(dict(error='Result not ready'), 404)
        if len(parts) == 6 and parts[4] == 'point' and parts[5].isdecimal():
            points = orjson.loads((directory / 'data.json').read_bytes())['points']
            index = int(parts[5])
            return self.respond(points[index]) if index < len(points) else self.respond(dict(error='Unknown point'), 404)
        if len(parts) == 5 and parts[4] in ('chart.json','data.json'):
            data=orjson.loads((directory/'data.json').read_bytes())
            data.pop('climb_route',None)
            if 'energy_guide' not in data:data['energy_guide']=energy_guide(data)
            return self.respond(chart_payload(data) if parts[4]=='chart.json' else data)
        if len(parts) == 5 and parts[4] == 'samples.csv':
            return self.file(directory / parts[4])
        if len(parts) == 5 and parts[4] == 'surface.json':
            data=orjson.loads((directory/'data.json').read_bytes())
            return self.respond(data.get('surface'))
        if len(parts) == 5 and parts[4] in ('diagram.svg', 'diagram.png', 'diagram.pdf'):
            query=parse_qs(urlparse(self.path).query)
            hidden=query.get('guide_visible')==['0']
            suffix=hashlib.sha256(REVISION.encode()+dump(hidden)).hexdigest()[:16]
            target=directory/('energy-'+suffix+'-'+parts[4])
            with FIGURE_LOCK:
                if not target.exists():
                    temporary = target.with_name('temporary-' + target.name)
                    data=orjson.loads((directory / 'data.json').read_bytes())
                    data['energy_guide']=None if hidden else data.get('energy_guide') or energy_guide(data)
                    export_figure(data, temporary)
                    temporary.replace(target)
            return self.file(target)
        return self.respond(dict(error='Not found'), 404)

    def do_POST(self):
        origin = self.headers.get('Origin')
        if origin and origin not in (f'http://127.0.0.1:{self.server.server_port}', f'http://localhost:{self.server.server_port}'):
            return self.respond(dict(error='Same-origin request required'), 403)
        try:
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 <= length <= 32768:
                raise ValueError('Request too large')
            body = json.loads(self.rfile.read(length) or b'{}')
            if not isinstance(body, dict):
                raise ValueError('Expected JSON object')
            path = urlparse(self.path).path
            if path == '/api/altitude/jobs':
                return self.respond(dict(id=start_job(body)), 202)
            parts = path.strip('/').split('/')
            if len(parts) == 5 and parts[:3] == ['api', 'altitude', 'jobs'] and JOB_ID.fullmatch(parts[3]) and parts[4] == 'cancel':
                with LOCK:
                    job = JOBS.get(parts[3])
                    active = bool(job and job['status'] in ('queued', 'running', 'exporting'))
                    if active:
                        job['cancel'].set()
                        if job['status'] == 'queued':
                            job['status'] = 'cancelled'
                return self.respond(dict(cancelled=active))
            return self.respond(dict(error='Not found'), 404)
        except (ValueError, KeyError, TypeError) as error:
            return self.respond(dict(error=str(error)), 400)


def main():
    global REVISION, POOL, POOL_STOP, INPUT_SIGNATURE
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8766)
    parser.add_argument('--open', action='store_true')
    args = parser.parse_args()
    url = f'http://127.0.0.1:{args.port}'
    REVISION = fingerprint()
    INPUT_SIGNATURE = input_signature()
    try:
        server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    except OSError as error:
        if args.open and error.errno == errno.EADDRINUSE:
            with urlopen(url + '/api/health', timeout=2) as response:
                if json.load(response).get('feature') == 'altitude-sep-v1':
                    webbrowser.open(url); return
        raise
    from altitude_pool import create
    POOL,POOL_STOP=create(dict(AIRCRAFT))
    print('Altitude SEP plotter: ' + url, flush=True)
    if args.open:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        with LOCK:
            for job in JOBS.values():
                if 'cancel' in job:
                    job['cancel'].set()
        POOL_STOP.set()
        server.server_close()
        WORKER.shutdown(wait=True, cancel_futures=True)
        POOL.shutdown(wait=True,cancel_futures=True)


if __name__ == '__main__':
    main()
