"""Isolated launch-envelope job: stdin request, NDJSON ray/progress/result lines.

The job owns a pool of simulation processes. It stops when its parent service
process disappears, so an abandoned calculation does not keep the CPU busy.
"""
if __name__ == '__main__':
    import os as _os
    if _os.environ.get('WT_MISSILE_ENGINE', 'native') != 'python':
        import json as _json
        import subprocess as _subprocess
        import sys as _sys
        from missile_engine import command as _command
        try:
            _request = _json.load(_sys.stdin)
            if isinstance(_request, dict):
                _request['_parent_pid'] = _os.getpid()
                _request['_supervisor_pid'] = _os.getppid()
            _status = _subprocess.run(_command('envelope'), input=_json.dumps(_request),
                                      text=True, encoding='utf-8').returncode
        except (RuntimeError, OSError, ValueError) as _error:
            print(_json.dumps(dict(type='error', error=f'{type(_error).__name__}: {_error}')), flush=True)
            _status = 1
        raise SystemExit(_status)

import json
import os
import sys

import missile_envelope


def parent_alive(pid):
    if pid is None: return True
    if os.name == 'nt':
        import ctypes
        kernel = ctypes.windll.kernel32
        handle = kernel.OpenProcess(0x00100000, False, pid)  # SYNCHRONIZE
        if not handle: return False
        try: return kernel.WaitForSingleObject(handle, 0) == 0x00000102  # WAIT_TIMEOUT
        finally: kernel.CloseHandle(handle)
    return os.getppid() == pid


def main():
    def send(kind, **data):
        print(json.dumps(dict(type=kind, **data), allow_nan=False), flush=True)
    try:
        request = json.load(sys.stdin)
        config = missile_envelope.validate(request)
        parent = request.get('_parent_pid') if isinstance(request.get('_parent_pid'), int) else None
        result = missile_envelope.search(
            config, emit=lambda kind, data: send(kind, **{kind: data}),
            cancelled=lambda: not parent_alive(parent))
        result.pop('rays')
        send('result', result=result)
    except Exception as error:
        send('error', error=f'{type(error).__name__}: {error}')
        raise SystemExit(1)


if __name__ == '__main__': main()
