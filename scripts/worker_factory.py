"""Warm data-independent libraries once, then fork isolated calculation workers.

This private Unix socket never serves HTTP. Dataset/model modules are imported
only in the child after its immutable input root has been selected.
"""
import json
import os
from pathlib import Path
import runpy
import signal
import socket
import sys

os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
os.environ.setdefault('OMP_NUM_THREADS','1')
os.environ.setdefault('MKL_NUM_THREADS','1')
import numpy
import scipy.optimize
import scipy.interpolate
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot
import contourpy
import orjson
from em_backend import activate,pin_session
assert activate()=='compiled'
pin_session('compiled')  # The immutable release's native binaries were checked above.
assert 'input_paths' not in sys.modules and 'aircraft_catalog' not in sys.modules
assert len(os.listdir('/proc/self/task'))==1, 'The worker template must be single-threaded'

endpoint=os.environ['WT_EM_WORKER_SOCKET']
if os.path.exists(endpoint):os.unlink(endpoint)
listener=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
listener.bind(endpoint);os.chmod(endpoint,0o600);listener.listen(3)
children=set()


def reap(*unused):
    while True:
        try:pid,_=os.waitpid(-1,os.WNOHANG)
        except ChildProcessError:return
        if not pid:return
        children.discard(pid)


signal.signal(signal.SIGCHLD,reap)
print(json.dumps(dict(ready=True)),flush=True)
while True:
    connection,_=listener.accept()
    try:
        connection.settimeout(5)
        # Consume exactly the handshake; later RPC bytes remain in the socket.
        line=bytearray()
        while len(line)<8192:
            value=connection.recv(1)
            if not value:raise ValueError('Worker handshake ended')
            if value==b'\n':break
            line.extend(value)
        else:raise ValueError('Worker handshake too large')
        command=json.loads(line);reap()
        if len(children)>=3:raise ValueError('Calculation worker capacity reached')
        connection.settimeout(None)
        # Do not let a very short-lived child be reaped before it is registered.
        blocked=signal.pthread_sigmask(signal.SIG_BLOCK,{signal.SIGCHLD})
        try:pid=os.fork()
        except BaseException:
            signal.pthread_sigmask(signal.SIG_SETMASK,blocked)
            raise
        if pid:
            children.add(pid)
            signal.pthread_sigmask(signal.SIG_SETMASK,blocked)
            connection.close();continue
        listener.close();signal.signal(signal.SIGCHLD,signal.SIG_DFL)
        signal.pthread_sigmask(signal.SIG_SETMASK,blocked)
        os.environ.update(command['environment'])
        os.environ.pop('WT_EM_SESSION_SOURCE',None)
        os.dup2(connection.fileno(),0);os.dup2(connection.fileno(),1);connection.close()
        print(json.dumps(dict(ready=True,pid=os.getpid())),flush=True)
        try:
            runpy.run_path(str(Path(__file__).with_name('worker_rpc.py')),run_name='__main__')
            sys.stdout.flush();sys.stderr.flush();os._exit(0)
        except BaseException:
            import traceback
            traceback.print_exc();sys.stderr.flush();os._exit(1)
    except Exception as error:
        print(str(error),file=sys.stderr,flush=True);connection.close()
