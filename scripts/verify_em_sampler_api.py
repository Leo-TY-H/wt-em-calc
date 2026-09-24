"""Time a saved configuration through the actual local HTTP API."""
import argparse
import json
import time
from pathlib import Path
from urllib.request import Request, urlopen


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True)
    parser.add_argument('--source', default='outputs/em/4a9cad9041b42dc7d783/data.json')
    parser.add_argument('--url', default='http://127.0.0.1:8765')
    parser.add_argument('--deadline', type=float, default=60.)
    parser.add_argument('--fresh-output', help='Assert that this server output directory is empty before submitting')
    args = parser.parse_args()
    source = json.loads(Path(args.source).read_text())
    config = source['settings']
    # Only settings are inputs. No saved equilibrium or interpolation data
    # are sent to the server.
    if args.fresh_output:
        directory = Path(args.fresh_output)
        assert directory.is_dir() and not any(directory.iterdir()), directory
    base = args.url.rstrip('/')
    start = time.monotonic()
    request = Request(base + '/api/jobs', json.dumps(config).encode(),
                      {'Content-Type': 'application/json'})
    with urlopen(request, timeout=10) as response:
        key = json.load(response)['id']
    last = None
    while True:
        with urlopen(base + '/api/jobs/' + key, timeout=10) as response:
            job = json.load(response)
        phase = (job.get('progress') or {}).get('phase', job['status'])
        if phase != last:
            print(round(time.monotonic() - start, 2), phase, flush=True)
            last = phase
        if job['status'] in ('complete', 'error', 'cancelled'):
            break
        if time.monotonic() - start > args.deadline:
            cancel = Request(base + '/api/jobs/' + key + '/cancel', b'{}',
                             {'Content-Type': 'application/json'})
            with urlopen(cancel, timeout=10):
                pass
            result = dict(status='timeout', job_id=key, elapsed_s=time.monotonic()-start,
                          meets_20s=False, meets_60s=False, settings=config, job=job,
                          source=args.source, url=base)
            Path(args.out).write_text(json.dumps(result, indent=2) + '\n')
            raise TimeoutError('HTTP benchmark exceeded its diagnostic deadline')
        time.sleep(.3)
    ready = time.monotonic() - start
    assert job['status'] == 'complete', job
    with urlopen(base + '/api/jobs/' + key + '/chart.json', timeout=10) as response:
        chart = json.load(response)
    assert len(chart['aircraft']) == len(config.get('entries') or config['aircraft'])
    assert not job.get('cached') and not chart.get('cached_aircraft') and not chart.get('cached_columns'), 'Cached work is not a cold benchmark'
    result = dict(status='complete', job_id=key, request_to_ready_s=ready,
                  chart_fetched_s=time.monotonic() - start, job=job,
                  meets_20s=ready <= 20, meets_60s=ready < 60,
                  source=args.source, settings=config, url=base,
                  empty_output_checked=bool(args.fresh_output),
                  note='Actual request through chart availability; numerical cache hits rejected. The source supplies settings only.')
    Path(args.out).write_text(json.dumps(result, indent=2) + '\n')
    print('READY', key, round(ready, 3), 'seconds', flush=True)


if __name__ == '__main__':
    main()
