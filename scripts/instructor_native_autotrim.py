"""Exact original mode-0 execution for the not-yet-translated asymmetric branch.

Offline Unicorn execution from the pinned local aces binary, using the existing
prepared-property/libm adapters. No live game, binary patch, or telemetry. This
is an explicit machine-code backend, NOT an independent Python reconstruction.
Only selected intact mode-0 inputs are supported. A changed binary fails closed.
"""
import copy,struct,threading
from collections import OrderedDict

SHA='820fee4a55601ffa459e2635da4ce1436cfc7adcc703a7c799380c95d136ebd5'
_machine=None
_model=None
_cache=OrderedDict()
_lock=threading.RLock()


def predict(model,ip,state,history):
    with _lock:return _predict(model,ip,state,history)


def _predict(model,ip,state,history):
    global _machine,_model
    from instructor_native import InstructorNative,BASE,OWNER,INPUT,OUTPUT,ARENA,HEAP
    from unicorn.x86_const import UC_X86_REG_RAX
    from body_dynamics import G
    if _machine is None:
        from pathlib import Path
        binary=Path(__file__).resolve().parents[1]/'references/native'/('aces-'+SHA)
        _machine=InstructorNative(binary_path=str(binary) if binary.is_file() else None)
        if _machine.sha!=SHA:
            _machine=None
            raise ValueError('Asymmetric Instructor requires the validated local aces binary '+SHA)
    n=_machine
    if _model is not model:
        mass=dict(mass=ip[0x2c]/G,cog=[state['f'].get(o,0.) for o in [0x5320,0x5324,0x5328]],inertia=[state['pitch_inertia']]*3)
        n.setup(model,mass,speed=max(1.,ip[0x38]),sweep=ip[0x74])
        _model=model;_cache.clear()
    key=(tuple(sorted(ip.items())),tuple(sorted(state['f'].items())),tuple(sorted(state['flags'].items())),
         state['pitch_inertia'],state['engine_count'],state['balance_multiplier'],state['new_balance'],state['rho0'],tuple(history))
    if key in _cache:return copy.deepcopy(_cache[key])
    data=bytearray(0xa0)
    for off,v in ip.items():
        if off==0:struct.pack_into('<I',data,off,v)
        elif off in (0xc,0x28,0x68):data[off]=int(v)
        else:struct.pack_into('<f',data,off,v)
    struct.pack_into('<QQ',data,0x90,0x107d6fba0,HEAP+0x22b8)
    for off,v in state['f'].items():n.floats(BASE+off,[v])
    for off,v in state['flags'].items():n.u.mem_write(BASE+off,bytes([bool(v)]))
    n.doubles(BASE+0x5358,[state['pitch_inertia']]);n.u.mem_write(OWNER+0x25f78,struct.pack('<I',state['engine_count']))
    n.floats(0x107d6fbb4,[state['balance_multiplier']]);n.u.mem_write(0x107d6fbb8,bytes([state['new_balance']]))
    n.floats(0x107d6f790,[state['rho0']]);n.u.mem_write(INPUT,bytes(data))
    hist=ARENA+0x6000;n.floats(hist,history[:2]);n.u.mem_write(hist+8,bytes([history[2]]))
    n.invoke(0x101a5cac0,[BASE,INPUT,OUTPUT,hist])
    result=dict(output=n.read(OUTPUT,13),success=bool(n.u.reg_read(UC_X86_REG_RAX)&255),
                history=n.read(hist,2)+[bool(n.u.mem_read(hist+8,1)[0])])
    if len(_cache)>=4096:_cache.popitem(last=False)
    _cache[key]=copy.deepcopy(result)
    return result
