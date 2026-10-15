"""Pin Python equations and native libraries for an EM server session.

Spawned workers inherit the source snapshot, including imports made after an
edit. Original filenames preserve resource paths. The temporary snapshot holds
code only, never calculation results, and is removed when the server exits.
"""
import atexit
import hashlib
import importlib.abc
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile

ENVIRONMENT='WT_EM_SESSION_SOURCE'
_snapshot=None
_owned_path=None


class SourceSnapshot(importlib.abc.MetaPathFinder,importlib.abc.Loader):
    def __init__(self,sources,native):
        self.sources=sources
        self.native=native

    def find_spec(self,fullname,path=None,target=None):
        if fullname in self.native:
            return importlib.util.spec_from_file_location(fullname,self.native[fullname])
        if fullname in self.sources:
            filename,_=self.sources[fullname]
            return importlib.util.spec_from_file_location(fullname,filename,loader=self)
        return None

    def create_module(self,spec):return None

    def exec_module(self,module):
        filename,source=self.sources[module.__name__]
        exec(compile(source,filename,'exec'),module.__dict__)


def cleanup():
    if _owned_path is not None:_owned_path.unlink(missing_ok=True)


def bootstrap():
    global _snapshot,_owned_path
    if _snapshot is not None:return
    inherited=os.environ.get(ENVIRONMENT)
    if inherited:
        snapshot=json.loads(Path(inherited).read_text(encoding='utf-8'))
    else:
        from em_backend import activate,MODULES
        kind=activate()
        import sysconfig
        suffix=sysconfig.get_config_var('EXT_SUFFIX')
        native={name:str(Path(sys.path[0])/(name+suffix)) for name in MODULES} if kind=='compiled' else {}
        folder=Path(__file__).resolve().parent
        sources={p.stem:[str(p),p.read_text(encoding='utf-8')] for p in folder.glob('*.py')}
        hashes={name:hashlib.sha256(Path(path).read_bytes()).hexdigest() for name,path in native.items()}
        snapshot=dict(sources=sources,native=native,hashes=hashes,backend=kind)
        with tempfile.NamedTemporaryFile(mode='w',encoding='utf-8',prefix='wt-em-session-',suffix='.json',delete=False) as out:
            json.dump(snapshot,out)
            _owned_path=Path(out.name)
        os.environ[ENVIRONMENT]=str(_owned_path)
        atexit.register(cleanup)
    if inherited:
        for name,path in snapshot['native'].items():
            if hashlib.sha256(Path(path).read_bytes()).hexdigest()!=snapshot['hashes'][name]:
                raise RuntimeError('Session solver file is missing or changed: '+name)
    sys.meta_path.insert(0,SourceSnapshot(snapshot['sources'],snapshot['native']))
    from em_backend import pin_session
    pin_session(snapshot['backend'])
    _snapshot=snapshot


def fresh_environment():
    env=dict(os.environ)
    env.pop(ENVIRONMENT,None)
    return env
