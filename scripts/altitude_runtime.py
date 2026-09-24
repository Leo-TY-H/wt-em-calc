"""Use the current equation backend without rebuilding another task's files."""
import json
import os
from em_backend import DIRECTORY, signature


def prepare():
    # Windows launches directly from the workspace; the portable backend
    # lock serializes any rebuild instead of silently selecting slow Python.
    if os.name == 'nt':
        return
    if os.environ.get('WT_EM_BACKEND'):
        return
    try:
        current = json.loads((DIRECTORY / 'manifest.json').read_text())['signature'] == signature()
    except (OSError, KeyError, ValueError):
        current = False
    # The launcher owns its snapshot and its .native_em directory. It can
    # safely rebuild there after model changes without touching the shared
    # backend or silently turning new calculations into slow Python runs.
    private_snapshot = (DIRECTORY.parent / 'snapshot.json').is_file()
    if not current and not private_snapshot:
        os.environ['WT_EM_BACKEND'] = 'python'


prepare()
