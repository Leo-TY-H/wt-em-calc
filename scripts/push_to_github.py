"""Commit local project changes and push, preserving concurrent remote updates."""
import argparse
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def run(*args, capture=False):
    return subprocess.run(args, cwd=ROOT, check=True, text=True, capture_output=capture)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--message', default='Update calculator source')
    args = parser.parse_args()
    branch = run('git', 'branch', '--show-current', capture=True).stdout.strip()
    if not branch:
        raise SystemExit('Check out a branch before pushing.')
    run('git', 'fetch', 'origin')
    # Commit local work before rebase; Git never has to discard uncommitted edits.
    run('git', 'add', '--all')
    from repository_policy import untrack_non_source,check
    untrack_non_source(ROOT)
    check(ROOT)
    if run('git', 'diff', '--cached', '--name-only', capture=True).stdout.strip():
        run('git', 'commit', '-m', args.message)
    exists = subprocess.run(['git', 'show-ref', '--verify', '--quiet', 'refs/remotes/origin/' + branch], cwd=ROOT)
    if exists.returncode == 0:
        try:
            run('git', 'rebase', 'origin/' + branch)
        except subprocess.CalledProcessError:
            run('git', 'rebase', '--abort')
            raise SystemExit('Remote edits overlap local changes. Your local commit is saved. Resolve the differences before pushing.')
    # Normal push rejects a race with another writer; never force-push.
    check(ROOT)
    run('git', 'push', '-u', 'origin', branch)


if __name__ == '__main__':
    main()
