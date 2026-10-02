"""Bounded missile jobs isolated from the existing EM calculation process."""
from concurrent.futures import ThreadPoolExecutor
from functools import lru_cache
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import uuid
from missile_names import display_name
from missile_inputs import directory as input_directory

ROOT = Path(__file__).resolve().parent
LOCK = threading.Lock()
MAX_WORKERS = min(4, max(1, int(os.environ.get('WT_MISSILE_WORKERS', min(4, max(1, (os.cpu_count() or 2)//2))))))
MAX_PENDING = 8
WORKER = ThreadPoolExecutor(max_workers=MAX_WORKERS, thread_name_prefix='missile-job')
JOBS = {}


@lru_cache(maxsize=1)
def metadata():
    inputs=input_directory(ROOT)
    index = json.loads((inputs / 'launch-profiles/index.json').read_text())
    entries = []
    for row in index['profiles']:
        key = row['path'].removesuffix('.json')
        if 'default' in key.lower().split('_'):
            continue
        profile = json.loads((inputs / 'launch-profiles' / row['path']).read_text())
        rocket = profile['properties']['rocket']; guidance = profile['properties']['guidance']
        # Presentation data comes from the same exported properties used by the
        # worker. Motor delays are measured on each motor's own controlled clock;
        # the guidance schedule is a gain ramp, not an extra blanket pause.
        timing = dict(motor_delays_s=[motor['delay'] for motor in guidance['motors']],
                      guidance_gain=[dict(time_s=point[0], gain=point[2]) for point in guidance['guidance']['time_gain']],
                      seeker_search_delay_s=guidance['manager']['lock_timeout'],
                      lock_after_launch=bool(rocket['guidance'].get('lockAfterLaunch',False)),
                      warm_up_s=rocket['guidance'].get('warmUpTime',0.),
                      proximity_delay_s=rocket.get('proximityFuse',{}).get('timeOut',.3) if rocket.get('hasProximityFuse',False) else None)
        entries.append(dict(id=key, name=display_name(key, rocket), family=row['family'],timing=timing))
    # Keep one base configuration for each displayed name, not mounting aliases.
    # The localization also labels AIM-4D as AIM-4G; prefer the matching G profile.
    unique = {}
    for entry in sorted(entries, key=lambda item: (-1 if item['id']=='us_aim4g_falcon' else len(item['id']), item['id'])):
        unique.setdefault(entry['name'].strip().casefold(), entry)
    entries = sorted(unique.values(), key=lambda item: (item['name'].casefold(), item['id']))
    return dict(missiles=entries, defaults=dict(missile='us_aim9l_sidewinder', duration=60,
                launcher=dict(position=[0,5000,0],velocity=[300,0,0],angles=[0,0,0]),
                target=dict(position=[4000,5000,0],velocity=[200,0,0],angles=[0,0,0])),
                model_status='experimental', profile_build='2.59.0.34',
                max_concurrent_jobs=MAX_WORKERS,
                assumptions='Ideal observability; geometric tracking limits; constant-velocity point target; flat ground; no wind.',
                units=dict(position='m',velocity='m/s',angles='degrees',time='s'),
                initialization='Launcher aircraft at release; coincident body-aligned mount; recovered launch processing and profile timing; ready prelaunch seeker with geometric limits and authored lock-before/after-launch behavior.')


def _run(key, config, cancel):
    process = None
    timer = None
    try:
        with LOCK:
            if cancel.is_set(): return
            JOBS[key].update(status='running', started=time.monotonic())
        process = subprocess.Popen([sys.executable, '-X', 'utf8', str(ROOT/'missile_worker.py')],
                    stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                    text=True, encoding='utf-8', creationflags=subprocess.CREATE_NO_WINDOW if os.name=='nt' else 0)
        with LOCK:
            JOBS[key]['process'] = process
            if cancel.is_set(): process.terminate()
        timer = threading.Timer(300, lambda: process.kill() if process.poll() is None else None)
        timer.daemon=True; timer.start()
        process.stdin.write(json.dumps(config)); process.stdin.close()
        completed = False
        for line in process.stdout:
            message = json.loads(line)
            with LOCK:
                if cancel.is_set(): continue
                if message['type']=='progress': JOBS[key]['progress']=message['progress']
                elif message['type']=='result':
                    JOBS[key].update(status='complete', result=message['result']); completed=True
                elif message['type']=='error': JOBS[key].update(status='failed', error=message['error'])
        process.wait()
        with LOCK:
            if not cancel.is_set() and not completed and JOBS[key]['status']!='failed':
                JOBS[key].update(status='failed',error='Simulation stopped before producing a result.')
    except Exception:
        with LOCK:
            if not cancel.is_set(): JOBS[key].update(status='failed',error='Unable to run missile simulation.')
    finally:
        if timer is not None: timer.cancel()
        if process is not None:
            if process.poll() is None: process.terminate()
            process.wait()
        with LOCK:
            JOBS[key].pop('process',None)
            JOBS[key]['finished']=time.monotonic()


def start(config):
    if not isinstance(config,dict): raise ValueError('Expected a scenario object')
    if config.get('missile') not in {m['id'] for m in metadata()['missiles']}: raise ValueError('Select a supported missile')
    with LOCK:
        if sum(job['status'] in ('queued','running') for job in JOBS.values())>=MAX_PENDING:
            raise ValueError('Missile simulation queue is full. Wait for the current run to finish.')
        old = sorted((key for key,j in JOBS.items() if 'finished' in j), key=lambda key:JOBS[key]['finished'])
        # Parallel short jobs can finish before their client's next poll.
        # Retain several full comparisons so another submission cannot erase them.
        for key in old[:-32]: del JOBS[key]
        key=uuid.uuid4().hex; event=threading.Event()
        JOBS[key]=dict(id=key,status='queued',cancel=event)
        WORKER.submit(_run,key,config,event)
    return key


def get(key):
    with LOCK:
        job=JOBS.get(key)
        if job is None: return None
        return {k:v for k,v in job.items() if k not in ('cancel','process','started','finished')}


def cancel(key):
    with LOCK:
        job=JOBS.get(key)
        if job is None or job['status'] not in ('queued','running'): return False
        job['cancel'].set(); job['status']='cancelled'
        process=job.get('process')
        if process is not None and process.poll() is None: process.terminate()
    return True
