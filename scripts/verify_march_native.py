"""Re-verify the march chart's compiled loop against the Python model and record it.

  python scripts/verify_march_native.py [--aircraft NAME:sb|rb ...] [--stamp] [--out report.json]

Runs march charts with WT_MARCH_NATIVE_VERIFY=1: every native finish row, every
explicit-history operating point and every Instructor admission is compared
with the Python result exactly, and every native event correction is shadowed
by the Python corrector (agreement to rounding; decisions must match). The
charts follow the Python results in this mode.

With ``--stamp`` and no failure, march_native/verified.json is rewritten with
the current source hashes. The march chart uses the compiled loop only while
those hashes match, so run this after any change to the model or solver
modules it lists (em_march_native.closure()).
"""
import argparse
import json
import os
import sys
import time
from pathlib import Path

os.environ['WT_MARCH_NATIVE_VERIFY'] = '1'
for key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):os.environ[key] = '1'
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
DEFAULT = ('bi:sb', 'f_16xl:rb', 'f_16a_block_10:sb', 'a6m2_zero:sb', 'ki_61_1a_otsu:rb')
NUMERIC = 1e-8  # Instructor trim and event corrections agree to rounding (S2/S3: <= 2e-12)


def run(spec):
    from em_solver import settings, compute
    name, mode = spec.split(':')
    rb = mode == 'rb'
    config = settings(dict(aircraft=[name], instructor=rb, torque_gyro=not rb, chart_solver='march'))
    t0 = time.perf_counter()
    data = compute(config)
    a = data['aircraft'][0]
    work = a.get('chart_solver_work') or {}
    return dict(aircraft=name, mode=mode, chart_solver=a.get('chart_solver'), fallback=a.get('chart_solver_fallback'),
                seconds=time.perf_counter()-t0, evaluations=work.get('evaluations'), native=work.get('native'))


def failures(row):
    out = []
    native = row.get('native') or {}
    if row['chart_solver'] != 'march':out.append('not run on the march chart: '+str(row.get('fallback')))
    if not str(native.get('status', '')).startswith('enabled'):out.append('native module not active: '+str(native.get('status')))
    verify = native.get('verify', {})
    if verify.get('operating_point', {}).get('mismatched'):out.append('operating point mismatch')
    for kind in ('instructor', 'event'):
        row_ = verify.get(kind, {})
        if row_.get('decision'):out.append(kind+' decision mismatch')
        if any(v > NUMERIC for v in row_.get('worst', {}).values()):out.append(kind+' value difference above '+str(NUMERIC))
    if native.get('curve_verify_mismatch'):out.append('finish row mismatch')
    if native.get('curve_propulsion_mismatch'):out.append('propulsion record mismatch')
    if not native.get('curve_verified'):out.append('no finish rows verified')
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--aircraft', action='append', help='NAME:sb or NAME:rb (default: one aircraft per propulsion/Instructor class)')
    ap.add_argument('--stamp', action='store_true');ap.add_argument('--out')
    args = ap.parse_args()
    rows = [run(spec) for spec in (args.aircraft or DEFAULT)]
    for row in rows:row['failures'] = failures(row)
    ok = all(not row['failures'] for row in rows)
    report = dict(passed=ok, aircraft=rows)
    if args.out:Path(args.out).write_text(json.dumps(report, indent=1, default=str))
    print(json.dumps([dict(aircraft=r['aircraft'], mode=r['mode'], failures=r['failures'], seconds=round(r['seconds'], 1),
                           verified=(r.get('native') or {}).get('verify')) for r in rows], default=str, indent=1))
    if args.stamp:
        if not ok:raise SystemExit('Not stamped: verification failed')
        import em_march_native as N
        record = N.stamp_record(dict(date=time.strftime('%Y-%m-%d'), aircraft=[r['aircraft']+':'+r['mode'] for r in rows],
                                     checks={r['aircraft']+':'+r['mode']: (r.get('native') or {}).get('verify') for r in rows},
                                     finish_rows=sum((r.get('native') or {}).get('curve_verified', 0) for r in rows),
                                     method='scripts/verify_march_native.py'))
        N.STAMP.write_text(json.dumps(record, indent=1, default=str)+'\n', newline='\n')
        print('Stamped '+str(N.STAMP))
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':main()
