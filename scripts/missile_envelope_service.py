"""Bounded launch-envelope jobs, each in its own calculation process.

One envelope runs at a time; its native worker owns the simulation threads.
Completed directions stream into the job so the page can draw partial results.
Cancelling keeps the finished directions.
"""
from concurrent.futures import ThreadPoolExecutor
import json
import os
import signal
import subprocess
import threading
import time
import uuid

import missile_engine

LOCK = threading.Lock()
MAX_PENDING = 2
TIME_LIMIT_S = float(os.environ.get('WT_ENVELOPE_TIME_LIMIT', 1800))
WORKER = ThreadPoolExecutor(max_workers=1, thread_name_prefix='missile-envelope')
JOBS = {}


def _kill_tree(process):
    if process is None or process.poll() is not None: return
    try:
        if os.name == 'nt':
            subprocess.run(['taskkill', '/PID', str(process.pid), '/T', '/F'], capture_output=True,
                           creationflags=subprocess.CREATE_NO_WINDOW, timeout=30)
        else:
            os.killpg(process.pid, signal.SIGTERM)
    except (OSError, subprocess.SubprocessError):
        pass
    if process.poll() is None:
        try: process.kill()
        except OSError: pass


def _run(key, config, cancel):
    process = None
    timer = None
    try:
        with LOCK:
            if cancel.is_set(): return
            JOBS[key].update(status='running', started_at=time.time())
        process = subprocess.Popen(missile_engine.command('envelope'),
                                   stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                   text=True, encoding='utf-8',
                                   creationflags=subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0,
                                   start_new_session=os.name != 'nt')
        with LOCK:
            JOBS[key]['process'] = process
            if cancel.is_set(): _kill_tree(process)
        def expire():
            with LOCK:
                if JOBS[key]['status'] == 'running': JOBS[key]['timed_out'] = True
            _kill_tree(process)
        timer = threading.Timer(TIME_LIMIT_S, expire); timer.daemon = True; timer.start()
        process.stdin.write(json.dumps(dict(config, _parent_pid=os.getpid()))); process.stdin.close()
        completed = False
        for line in process.stdout:
            message = json.loads(line)
            with LOCK:
                job = JOBS[key]
                if message['type'] == 'ray': job['rays'].append(message['ray'])
                elif message['type'] == 'plan': job['plan'] = message['plan']
                elif message['type'] == 'progress': job['progress'] = message['progress']
                elif cancel.is_set(): continue
                elif message['type'] == 'result':
                    job.update(status='complete', summary=message['result']); completed = True
                elif message['type'] == 'error': job.update(status='failed', error=message['error'])
        process.wait()
        with LOCK:
            job = JOBS[key]
            if not cancel.is_set() and not completed and job['status'] != 'failed':
                job.update(status='failed', error=('Envelope calculation reached its time limit; finished directions are kept.'
                                                   if job.get('timed_out') else 'Envelope calculation stopped before finishing.'))
    except Exception as error:
        with LOCK:
            if not cancel.is_set(): JOBS[key].update(status='failed', error=f'Unable to run the envelope calculation: {error}')
    finally:
        if timer is not None: timer.cancel()
        _kill_tree(process)
        if process is not None:
            process.wait()
            for stream in (process.stdin, process.stdout):
                try: stream.close()
                except OSError: pass  # A cancelled worker may have closed its pipe.
        with LOCK:
            JOBS[key].pop('process', None)
            JOBS[key]['finished'] = time.monotonic()


def start(request):
    if os.environ.get('WT_MISSILE_ENGINE', 'native') == 'python':
        import missile_envelope
        config = missile_envelope.validate(request)
        angles = missile_envelope.azimuths(config)
        setup = dict(config=config, azimuths_deg=angles, mirrored=missile_envelope.mirrored(config),
                     rays_total=len(config['layers_m']) * len(angles))
    else:
        try:
            setup = missile_engine.calculate('envelope-setup', request)
        except (RuntimeError, OSError) as error:
            # The HTTP endpoint reports validation and unavailable-build errors
            # through ValueError, without running a reference calculation.
            raise ValueError(str(error)) from error
        config = setup['config']
    with LOCK:
        if sum(job['status'] in ('queued', 'running') for job in JOBS.values()) >= MAX_PENDING:
            raise ValueError('An envelope calculation is already running. Wait for it to finish or stop it.')
        old = sorted((k for k, j in JOBS.items() if 'finished' in j), key=lambda k: JOBS[k]['finished'])
        for k in old[:-4]: del JOBS[k]
        key = uuid.uuid4().hex; event = threading.Event()
        JOBS[key] = dict(id=key, status='queued', cancel=event, config=config, rays=[], progress=None,
                         rays_total=setup['rays_total'],
                         mirrored=setup['mirrored'], azimuths_deg=setup['azimuths_deg'],
                         created_at=time.time())
        WORKER.submit(_run, key, config, event)
    return key


def get(key, since=0):
    with LOCK:
        job = JOBS.get(key)
        if job is None: return None
        out = {k: v for k, v in job.items() if k not in ('cancel', 'process', 'finished', 'rays', 'timed_out')}
        out['ray_count'] = len(job['rays'])
        out['rays'] = job['rays'][max(0, since):]
        out['since'] = max(0, since)
        return out


def cancel(key):
    with LOCK:
        job = JOBS.get(key)
        if job is None or job['status'] not in ('queued', 'running'): return False
        job['cancel'].set(); job['status'] = 'cancelled'
        process = job.get('process')
    _kill_tree(process)
    return True


def busy():
    with LOCK: return any(job['status'] in ('queued', 'running') for job in JOBS.values())
