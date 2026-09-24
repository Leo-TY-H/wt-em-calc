"""Selected full-real manual-engine snapshot consumer101a4e5f0.

Intact running, no start/stop/autopilot/feather requests. Only auto-prop is
optional; water/oil radiator auto and mixture auto are disabled explicitly.
"""
from component_assembly import f32,mul
from primary_controls import delivered_commands
from engine_commands import afterburner_command

def quantize(x,scale):
    return mul(float(min(255,max(0,int(mul(f32(x),float(scale)))))),f32(1./scale))

def deliver(cp,pp,snapshot,state,ranges,dt):
    e=dict(state['engine']);e.setdefault('gear',0);e.setdefault('regulator',-1.);e['throttle']=f32(snapshot['throttle'])
    e['afterburner']=afterburner_command(snapshot['afterburner'],pp['engine']['boost_type'],pp['engine']['boost_controllable'])
    e['mixture']=quantize(snapshot['mixture'],200) if pp['engine']['mixer_type']==2 else f32(e.get('mixture',.5))
    if pp['engine']['manual_compressor'] and snapshot['gear']!=e.get('gear',0):e.update(gear=snapshot['gear'],regulator=-1.)
    return dict(delivered=delivered_commands(cp,snapshot['commands'],state['delivered'],ranges,dt),
        trim_requested=[x if y else 0. for x,y in zip(state['trim_requested'],cp['trim_available'])],
        trim_actual=[x if y else 0. for x,y in zip(state['trim_actual'],cp['trim_available'])],
        engine=e,command=state.get('command',1.) if snapshot['auto'] else quantize(snapshot['command'],255),auto=bool(snapshot['auto']),
        radiator=quantize(snapshot['radiator'],255),oil_radiator=quantize(snapshot['oil_radiator'],255))
