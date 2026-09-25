"""Enforce a source-only Git tree; runtime data and results stay local."""
from pathlib import PurePosixPath
import subprocess

ROOT_FILES={'.gitignore','.gitattributes','.dockerignore','Dockerfile','fly.toml',
            'requirements.txt','requirements-plotter.txt'}


def is_source(name):
    path=PurePosixPath(name)
    if len(path.parts)==1:return name in ROOT_FILES or path.suffix=='.cmd'
    if path.parts[0] in ('scripts','tests'):return len(path.parts)==2 and path.suffix=='.py'
    if path.parts[0]=='app':return path.suffix in ('.js','.css','.html') and len(path.parts)<=3
    if path.parts[:2]==('deploy','oracle'):
        return len(path.parts)==3 and path.suffix in ('.py','.sh','.md')
    return path.parts[:2]==('.github','workflows') and len(path.parts)==3 and path.suffix in ('.yml','.yaml')


def tracked(root):
    return [p.decode('utf-8') for p in subprocess.check_output(['git','ls-files','-z'],cwd=root).split(b'\0') if p]


def untrack_non_source(root):
    paths=[p for p in tracked(root) if not is_source(p)]
    if paths:
        payload=b'\0'.join((':(literal)'+p).encode('utf-8') for p in paths)+b'\0'
        subprocess.run(['git','rm','--cached','--ignore-unmatch','--quiet',
            '--pathspec-from-file=-','--pathspec-file-nul'],input=payload,cwd=root,check=True)
    return paths


def check(root):
    invalid=[p for p in tracked(root) if not is_source(p)]
    if invalid:raise ValueError('Non-source files in Git index: '+', '.join(invalid[:10]))


if __name__=='__main__':
    from pathlib import Path
    check(Path(__file__).resolve().parents[1])
