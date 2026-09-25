"""Isolated cold-chart comparison for worker counts and Instructor compilation.

Run in a disposable copy/container, never the live API process. Each invocation
uses fresh workers and writes evidence outside the production results directory.
"""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import time

ROOT=Path(__file__).resolve().parents[1]
# Spawn runs this module in each worker too, so the reference selection applies
# consistently to parent and workers, without altering the production backend.
if os.environ.get('EM_BENCH_REFERENCE')=='1':
    import sys
    from em_backend import activate
    activate()
    for name in ('instructor_aoa_balance','windows_instructor_source','instructor_chart_inputs','instructor_aoa'):
        spec=importlib.util.spec_from_file_location(name,ROOT/'scripts'/(name+'.py'))
        module=importlib.util.module_from_spec(spec)
        sys.modules[name]=module
        spec.loader.exec_module(module)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--aircraft',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    import resource
    import orjson
    import numpy as np
    import em_workers
    from em_solver import compute,settings
    from em_plot import smooth_surface,contour_paths
    # Use the same start method in all benchmark variants, including reference
    # helpers. Production continues to use its existing persistent forkserver.
    em_workers.START_METHOD='spawn'
    config=settings(dict(aircraft=[args.aircraft],roll_leveling=False,
        sep_tolerance_mps=.15,load_samples=13,heatmap=False))
    initial=resource.getrusage(resource.RUSAGE_SELF)
    initial_children=resource.getrusage(resource.RUSAGE_CHILDREN)
    initial_cpu=initial.ru_utime+initial.ru_stime+initial_children.ru_utime+initial_children.ru_stime
    started=time.perf_counter()
    result=compute(config)
    compute_s=time.perf_counter()-started
    for aircraft in result['aircraft']:smooth_surface(result,aircraft)
    contours=contour_paths(result)
    def digest(value):
        return hashlib.sha256(orjson.dumps(value,option=orjson.OPT_SERIALIZE_NUMPY|orjson.OPT_SORT_KEYS)).hexdigest()
    evidence=[];numerics=[]
    for aircraft in result['aircraft']:
        fields=('speed_kmh','load_g','turn_dps','ps_mps','alpha_deg','valid','reasons',
                'force_error_g','angular_error_rad_s2')
        points=[{k:p.get(k) for k in fields} for p in aircraft['points']]
        points.sort(key=lambda p:(p['speed_kmh'],p['load_g'],p['alpha_deg']))
        surface=aircraft['surface']
        numerics.append(dict(id=aircraft['id'],points=points,boundary=aircraft['boundary'],
            surface={k:surface[k] for k in ('x','fraction','z')},
            contours=contours[aircraft['id']],
            interpolation={k:aircraft['interpolation'][k] for k in
                           ('target_speed_kmh','target_contour_dps','unresolved_speed_intervals')},
            columns=[dict(speed_kmh=c['speed_kmh'],boundary={k:c['boundary'].get(k) for k in fields}
                          if c.get('boundary') else None,boundary_status=c['boundary_status'])
                     for c in aircraft['columns']]))
        accepted=[p for p in points if p['valid']]
        evidence.append(dict(id=aircraft['id'],points=digest(points),boundary=digest(aircraft['boundary']),
            surface=digest(surface),contours=digest(contours[aircraft['id']]),
            mask=digest({k:np.isfinite(np.asarray(v,dtype=float)) for k,v in surface.items() if isinstance(v,list)}),
            columns=len(aircraft['columns']),point_count=len(points),
            max_force_error_g=max((p['force_error_g'] for p in accepted),default=0.),
            max_angular_error_rad_s2=max((p['angular_error_rad_s2'] for p in accepted),default=0.)))
    em_workers.shutdown()
    usage=resource.getrusage(resource.RUSAGE_SELF)
    children=resource.getrusage(resource.RUSAGE_CHILDREN)
    elapsed=time.perf_counter()-started
    report=dict(aircraft=args.aircraft,workers=em_workers.WORKERS,reference=os.environ.get('EM_BENCH_REFERENCE')=='1',
        compute_s=compute_s,total_s=elapsed,cpu_s=usage.ru_utime+usage.ru_stime+children.ru_utime+children.ru_stime-initial_cpu,
        settings=config,start_method=em_workers.START_METHOD,blas_threads=os.environ.get('OPENBLAS_NUM_THREADS'),
        parent_peak_rss_kib=usage.ru_maxrss,child_peak_rss_kib=children.ru_maxrss,evidence=evidence)
    peak=Path('/sys/fs/cgroup/memory.peak')
    if peak.exists():report['container_peak_memory_bytes']=int(peak.read_text())
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    import gzip
    with gzip.open(args.output.with_suffix('.numerics.json.gz'),'wb') as stream:
        stream.write(orjson.dumps(numerics,option=orjson.OPT_SERIALIZE_NUMPY))
    print(json.dumps({k:v for k,v in report.items() if k not in ('evidence','settings')}),flush=True)


if __name__=='__main__':main()
