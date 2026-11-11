"""Immutable inputs selected for one calculation worker, independently of solver code.

The service validates the version and passes its root before importing any model.
Workers never change this root or share module caches between input versions.
"""
import os
from pathlib import Path

CODE_ROOT = Path(__file__).resolve().parents[1]
ROOT = Path(os.environ.get('WT_EM_INPUT_ROOT', CODE_ROOT)).resolve()
REFERENCES = ROOT / 'references'


def policy(name):
    """Release policies follow the solver; authored and compiled FM inputs follow the version."""
    return CODE_ROOT / 'references' / name
