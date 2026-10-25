"""Bounded benchmark artifacts; independent of solver work and admission policy.

Writers sharing a workstream root serialize reservations with an OS file lock.
Limits include existing files and temporary writes. No files are auto-pruned.
"""
from contextlib import contextmanager
import gzip
import json
import math
import os
from pathlib import Path
import stat
import sys
import tempfile
import uuid

DEFAULTS = dict(summary=1_000_000, diagnostic=25_000_000, total=100_000_000)
CONTROL_LIMIT = 1_000_000


def linked(path):
    return path.is_symlink() or bool(getattr(path.lstat(), 'st_file_attributes', 0)
                                    & getattr(stat, 'FILE_ATTRIBUTE_REPARSE_POINT', 1024))


def add_arguments(parser):
    group = parser.add_argument_group('artifact storage (does not limit calculations)')
    for kind, size in DEFAULTS.items():
        group.add_argument('--output-'+kind+'-mb', type=float, default=None,
                           help=f'{kind} output budget in decimal MB (default {size/1e6:g})')
    group.add_argument('--output-budget-root', type=Path,
                       help='Shared workstream directory; all batch/worker outputs count together')
    group.add_argument('--output-budget-reason', help='Recorded justification for a larger budget')
    group.add_argument('--full-output', action='store_true',
                       help='Keep optional solver diagnostics too; byte limits still apply')


def default(value):
    if hasattr(value, 'tolist'):
        return value.tolist()
    if isinstance(value, Path):
        return str(value)
    raise TypeError(type(value).__name__)


def json_chunks(value):
    for chunk in json.JSONEncoder(default=default, separators=(',', ':')).iterencode(value):
        for start in range(0, len(chunk), 16384):
            yield chunk[start:start+16384].encode('utf-8')


# Reporting histories only. Never remove states, residuals, admission reasons,
# boundary certificates, validation records, work counters, or numerical arrays.
DIAGNOSTICS = frozenset(('stationary_attempts', 'stationary_candidates',
    'correction_diagnostics', 'search_diagnostics', 'search_evidence',
    'stationary_boundary_evidence', 'propeller_branch_recovery'))


def compact(value):
    if isinstance(value, dict):
        return {k: compact(v) for k, v in value.items() if k not in DIAGNOSTICS}
    if isinstance(value, (list, tuple)):
        return [compact(v) for v in value]
    return value


class LimitReached(Exception):
    pass


class LimitedStream:
    def __init__(self, stream, maximum):
        self.stream, self.maximum, self.count = stream, maximum, 0

    def write(self, data):
        if self.count+len(data) > self.maximum:
            raise LimitReached('artifact budget reached')
        size = self.stream.write(data)
        self.count += size
        return size

    def flush(self):
        self.stream.flush()

    def tell(self):
        return self.count


class OutputBudget:
    def __init__(self, root, run, limits=None, reason=None):
        self.root = Path(root).absolute()
        self.root.mkdir(parents=True, exist_ok=True)
        if self.root.resolve() != self.root or self.root.is_symlink():
            raise ValueError('Output root must be an independent directory, not a link/junction')
        self.run = str(run)
        self.limits = dict(DEFAULTS if limits is None else limits)
        if any(not math.isfinite(v) or v <= 0 for v in self.limits.values()):
            raise ValueError('Output limits must be finite and positive')
        self.reason = reason
        if any(self.limits[k] > DEFAULTS[k] for k in DEFAULTS) and not (reason or '').strip():
            raise ValueError('Larger output budgets require --output-budget-reason')
        self.control = self.root/'.benchmark-output.json'
        self.warned = set()
        with self.locked():
            state = self.load()
            previous = state.get('total_limit', self.limits['total'])
            # A caller may tighten the shared cap; raising it is explicit.
            if self.limits['total'] != previous and not (reason or '').strip():
                raise ValueError('Changing an existing shared budget requires a reason')
            state['total_limit'] = self.limits['total']
            state['runs'].setdefault(self.run, dict(missing={}, omitted=0))
            state['runs'][self.run].update(limits=self.limits, reason=reason,
                command=sys.argv, owner_pid=os.getpid())
            self.save(state)

    @classmethod
    def from_args(cls, args, output, directory=False):
        output = Path(output).absolute()
        inherited = json.loads(os.environ.get('WT_BENCHMARK_OUTPUT', '{}'))
        limits = inherited.get('limits', dict(DEFAULTS)).copy()
        for kind in DEFAULTS:
            value = getattr(args, 'output_'+kind+'_mb', None)
            if value is not None:
                if not math.isfinite(value) or value <= 0:
                    raise ValueError('Output limits must be finite and positive')
                limits[kind] = int(value*1_000_000)
        root = getattr(args, 'output_budget_root', None) or inherited.get('root')
        root = root or (output if directory else output.parent)
        run = inherited.get('run') if inherited.get('output') == str(output) else None
        budget = cls(root, run or (str(output)+'#'+uuid.uuid4().hex), limits,
                     getattr(args, 'output_budget_reason', None) or inherited.get('reason'))
        budget.output = str(output)
        return budget

    def export(self):
        """Pass the same workstream cap to explicitly launched child processes."""
        os.environ['WT_BENCHMARK_OUTPUT'] = json.dumps(dict(root=str(self.root),
            limits=self.limits, reason=self.reason, run=self.run,
            output=getattr(self, 'output', None)))

    @contextmanager
    def locked(self):
        with (self.root/'.benchmark-output.lock').open('a+b') as stream:
            if stream.tell() == 0:
                stream.write(b'0'); stream.flush()
            stream.seek(0)
            if os.name == 'nt':
                import msvcrt
                import errno
                import time
                while True:
                    try:
                        msvcrt.locking(stream.fileno(), msvcrt.LK_NBLCK, 1)
                        break
                    except OSError as error:
                        if error.errno not in (errno.EACCES, errno.EAGAIN, errno.EDEADLK):
                            raise
                        time.sleep(.05)
            else:
                import fcntl
                fcntl.flock(stream, fcntl.LOCK_EX)
            try:
                yield
            finally:
                stream.seek(0)
                if os.name == 'nt':
                    msvcrt.locking(stream.fileno(), msvcrt.LK_UNLCK, 1)
                else:
                    fcntl.flock(stream, fcntl.LOCK_UN)

    def load(self):
        return json.loads(self.control.read_text(encoding='utf-8')) if self.control.exists() else dict(files={}, runs={})

    def save(self, state):
        data = json.dumps(state, separators=(',', ':')).encode('utf-8')
        if len(data) > CONTROL_LIMIT:
            raise LimitReached('Output ledger is full; use fewer files')
        temporary = self.control.with_suffix('.tmp')
        temporary.write_bytes(data)
        os.replace(temporary, self.control)

    def path(self, path):
        path = Path(path).absolute()
        if not path.is_relative_to(self.root) or path.resolve() != path:
            raise ValueError('Artifact must be inside the workstream root, without links/junctions')
        if path.name.startswith('.benchmark-output'):
            raise ValueError('Reserved output bookkeeping name')
        return path

    def usage(self):
        total = 0
        for base, dirs, files in os.walk(self.root, followlinks=False):
            for name in dirs+files:
                p = Path(base)/name
                if linked(p):
                    if name in dirs:
                        dirs.remove(name)
                    else:
                        files.remove(name)
            total += sum((Path(base)/name).stat().st_size for name in files
                         if not name.startswith('.benchmark-output'))
        return total

    def write(self, path, producer, kind='diagnostic', required=True, append=False):
        path = self.path(path)
        key = path.relative_to(self.root).as_posix()
        with self.locked():
            state = self.load()
            record = state['files'].get(key)
            if record and record['run'] != self.run:
                raise ValueError('Output belongs to another run: '+str(path))
            if record and record['kind'] != kind:
                raise ValueError('An artifact cannot change budget category: '+str(path))
            if path.exists() and not record:
                raise FileExistsError('Refusing to overwrite unowned output: '+str(path))
            old = path.stat().st_size if path.exists() else 0
            used = sum((self.root/name).stat().st_size for name, item in state['files'].items()
                       if item['run'] == self.run and item['kind'] == kind and (self.root/name).exists())
            # Reserve ledger and its atomic replacement; temporary data counts too.
            available = min(self.limits[kind]-used+(0 if append else old),
                            (state['total_limit']-self.usage()-2*CONTROL_LIMIT)//(2 if append else 1))
            path.parent.mkdir(parents=True, exist_ok=True)
            temporary = None
            try:
                if available < 0:
                    raise LimitReached('workstream budget reached')
                projected = dict(state['files'], **{key: dict(run=self.run, kind=kind)})
                if len(json.dumps(dict(state, files=projected))) > CONTROL_LIMIT-65536:
                    raise LimitReached('output ledger capacity reached; consolidate artifacts')
                with tempfile.NamedTemporaryFile(dir=path.parent, prefix='.bench-', delete=False) as stream:
                    temporary = Path(stream.name)
                    bounded = LimitedStream(stream, available)
                    producer(bounded)
                if append:
                    # Bounded payload is complete before appending: no broken JSONL records.
                    with path.open('ab') as target, temporary.open('rb') as source:
                        while chunk := source.read(65536):
                            target.write(chunk)
                else:
                    os.replace(temporary, path)
                state['files'][key] = dict(run=self.run, kind=kind)
                state['runs'][self.run]['missing'].pop(key, None)
                success = True
            except LimitReached:
                run = state['runs'][self.run]
                run['omitted'] += 1
                if required:
                    if len(run['missing']) < 128:
                        run['missing'][key] = True
                    else:
                        run['missing_overflow'] = True
                if key not in self.warned:
                    print('Output budget reached: '+str(path)+
                          '; calculation continues. Raise output limits with a reason if needed.', file=sys.stderr)
                    self.warned.add(key)
                success = False
            finally:
                if temporary is not None:
                    temporary.unlink(missing_ok=True)
            self.save(state)
            return success

    def json(self, path, value, kind='summary', required=True):
        def produce(stream):
            if str(path).endswith('.gz'):
                with gzip.GzipFile(fileobj=stream, mode='wb', mtime=0) as zipped:
                    for chunk in json_chunks(value):
                        zipped.write(chunk)
            else:
                for chunk in json_chunks(value):
                    stream.write(chunk)
        return self.write(path, produce, kind, required)

    def profile(self, path, profiler):
        import marshal
        profiler.create_stats()
        return self.write(path, lambda stream: marshal.dump(profiler.stats, stream), required=False)

    def drain(self, pipe, path):
        """Drain a child pipe even after the optional log reaches its cap."""
        import threading
        stats = dict(bytes_seen=0, bytes_saved=0)
        def consume():
            saving = True
            try:
                while chunk := pipe.read(16384):
                    stats['bytes_seen'] += len(chunk)
                    if saving:
                        try:
                            saving = self.write(path, lambda stream: stream.write(chunk),
                                                required=False, append=True)
                        except Exception as error:
                            stats['error'] = str(error); saving = False
                        if saving:
                            stats['bytes_saved'] += len(chunk)
            finally:
                pipe.close()
        thread = threading.Thread(target=consume, daemon=True)
        thread.start()
        return thread, stats

    def status(self):
        with self.locked():
            row = self.load()['runs'][self.run]
            return dict(evidence_incomplete=bool(row['missing'] or row.get('missing_overflow')),
                        detail_truncated=bool(row['omitted']), omitted_writes=row['omitted'])

    def finish(self):
        return 2 if self.status()['evidence_incomplete'] else 0


def read_json(path):
    path = Path(path)
    opener = gzip.open if path.suffix == '.gz' else open
    with opener(path, 'rt', encoding='utf-8') as stream:
        return json.load(stream)


if __name__ == '__main__':
    import argparse
    import subprocess
    parser = argparse.ArgumentParser(description='Run a benchmark with bounded subprocess logs')
    add_arguments(parser)
    parser.add_argument('--log', type=Path, required=True)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if not command:
        parser.error('Supply a command after --')
    budget = OutputBudget.from_args(args, args.log)
    budget.export()
    child = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    thread, stats = budget.drain(child.stdout, args.log)
    code = child.wait(); thread.join()
    budget.json(args.log.with_suffix('.storage.json'), dict(returncode=code, log=stats))
    raise SystemExit(code or budget.finish())
