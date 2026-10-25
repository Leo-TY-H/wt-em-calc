"""Application transport for the standalone native calculation executable.

This module performs no flight or envelope calculations. Reference mode is an
explicit testing/development option; a missing native build never silently
switches a production request back to Python calculations.
"""
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parent


def command(mode):
    setting = os.environ.get('WT_MISSILE_ENGINE', 'native')
    if setting not in ('native', 'python'):
        raise ValueError('Unknown WT_MISSILE_ENGINE')
    if mode not in ('simulator', 'envelope', 'envelope-setup'):
        raise ValueError('Unknown missile calculation mode')
    if setting == 'python':
        if mode == 'envelope-setup':
            raise ValueError('Envelope setup transport is native only')
        name = 'missile_worker.py' if mode == 'simulator' else 'missile_envelope_worker.py'
        return [sys.executable, '-X', 'utf8', str(ROOT / name)]
    from build_missile_engine import executable
    return [str(executable()), '--root', str(ROOT.parent), '--mode', mode]


def calculate(mode, request, emit=lambda kind, data: None):
    """Small synchronous transport used by integration checks and local tools."""
    process = subprocess.Popen(command(mode), stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE, text=True, encoding='utf-8',
                               creationflags=subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0)
    try:
        process.stdin.write(json.dumps(request)); process.stdin.close()
        result = None
        for line in process.stdout:
            message = json.loads(line)
            kind = message['type']
            if kind == 'error':
                raise RuntimeError(message['error'])
            if kind == 'result':
                result = message['result']
            else:
                emit(kind, message[kind])
        if process.wait() != 0 or result is None:
            raise RuntimeError('Native calculation stopped before producing a result')
        return result
    finally:
        if process.poll() is None:
            process.kill()
        process.wait()
        process.stdout.close(); process.stderr.close()
