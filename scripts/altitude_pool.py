"""Persistent isolated workers; startup is paid once, outside calculations."""
import multiprocessing
import os
from concurrent.futures import ProcessPoolExecutor


def initialize(catalog,event):
    from aircraft_catalog import install_worker_catalog
    import em_cancellation
    install_worker_catalog(catalog)
    em_cancellation.initialize(event)


def warm():
    import time
    time.sleep(.1)
    return os.getpid()


def create(catalog):
    context=multiprocessing.get_context('spawn')
    stop=context.Event()
    count=max(1,min(6,(os.cpu_count() or 2)//2))
    pool=ProcessPoolExecutor(max_workers=count,mp_context=context,initializer=initialize,initargs=(catalog,stop))
    for f in [pool.submit(warm) for _ in range(count)]:f.result()
    return pool,stop
