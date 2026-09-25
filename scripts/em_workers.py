"""Reuse numerical workers across entry batches and interactive requests."""
import atexit
import multiprocessing
import os
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
from contextlib import contextmanager
from threading import RLock
from em_cancellation import initialize

_lock=RLock()
_pool=None
_cancel=None
_depth=0
def cpu_budget():
    """Available CPUs, including affinity and Linux container bandwidth limits."""
    import math
    limits=[os.cpu_count() or 1]
    if hasattr(os,'sched_getaffinity'):
        try:limits.append(len(os.sched_getaffinity(0)))
        except OSError:pass
    # Containers normally expose their cgroup at this root. Include ancestor
    # quotas when the process belongs to a nested cgroup within the mount.
    root=Path('/sys/fs/cgroup')
    locations=[root]
    legacy=[root/'cpu',root/'cpu,cpuacct',root]
    def ancestors(mount,path):
        child=mount/path.lstrip('/')
        return [] if '..' in child.parts else [child,*[p for p in child.parents if mount in p.parents]]
    try:
        for line in Path('/proc/self/cgroup').read_text().splitlines():
            _,controllers,path=line.split(':',2)
            if not controllers:
                locations.extend(ancestors(root,path))
            elif 'cpu' in controllers.split(','):
                for mount in (root/'cpu',root/'cpu,cpuacct'):
                    legacy.extend(ancestors(mount,path))
    except (OSError,ValueError):pass
    for directory in locations:
        try:
            quota,period=(directory/'cpu.max').read_text().split()
            if quota!='max':limits.append(int(quota)/int(period))
        except (OSError,ValueError,ZeroDivisionError):pass
    for directory in legacy:
        try:
            quota=int((directory/'cpu.cfs_quota_us').read_text())
            period=int((directory/'cpu.cfs_period_us').read_text())
            if quota>0:limits.append(quota/period)
        except (OSError,ValueError,ZeroDivisionError):pass
    return max(1,math.floor(min(limits)))


def worker_count():
    budget=cpu_budget()
    requested=os.environ.get('WT_EM_WORKERS')
    # Desktop defaults retain one CPU for the interface. An explicit count
    # may use the full budget on a dedicated server.
    return max(1,min(int(requested),budget)) if requested is not None else max(1,min(12,budget-1))


WORKERS=worker_count()
START_METHOD=os.environ.get('WT_EM_PROCESS_START',
    'forkserver' if 'forkserver' in multiprocessing.get_all_start_methods() else 'spawn')


def initialize_worker(event,catalog):
    # A forkserver preloads the solver, then multiprocessing executes the
    # caller's main module. That module may put scripts/ ahead of the native
    # library again. Restore backend precedence before any lazy controller
    # imports; the parent's BACKEND label alone does not establish this.
    from em_backend import activate
    activate()
    initialize(event)
    from aircraft_catalog import install_worker_catalog
    install_worker_catalog(catalog)
    from em_sampling import _AIRCRAFT_CACHE,_COLUMN_CACHE,worker_solver
    _AIRCRAFT_CACHE.clear();_COLUMN_CACHE.clear();worker_solver.cache_clear()


def shutdown():
    global _pool,_cancel
    if _pool is not None:
        _cancel.set()
        _pool.shutdown(wait=True,cancel_futures=True)
        _pool=None;_cancel=None


@contextmanager
def process_pool():
    global _pool,_cancel,_depth
    with _lock:
        if _pool is None:
            method=START_METHOD
            context=multiprocessing.get_context(method)
            if method=='forkserver':
                # Import the immutable equation libraries once in a separate,
                # single-threaded owner, before it creates numerical workers.
                # No aircraft solver or evaluated condition is preloaded.
                # Python 3.9's forkserver does not apply its sys_path argument
                # before preloading, so provide the module path explicitly.
                directory=str(Path(__file__).resolve().parent)
                paths=os.environ.get('PYTHONPATH','').split(os.pathsep)
                os.environ['PYTHONPATH']=os.pathsep.join([directory,*[p for p in paths if p and p!=directory]])
                context.set_forkserver_preload(['em_solver','em_sampling'])
            _cancel=context.Event()
            from aircraft_catalog import catalog
            snapshot=catalog()
            _pool=ProcessPoolExecutor(max_workers=WORKERS,mp_context=context,
                initializer=initialize_worker,initargs=(_cancel,snapshot))
        _depth+=1
        try:yield _pool,_cancel
        finally:
            _depth-=1
            # Cancellation poisons only this request's pool. Drain the stopped
            # work before a later calculation can acquire fresh workers.
            if _depth==0 and _cancel.is_set():shutdown()


atexit.register(shutdown)
