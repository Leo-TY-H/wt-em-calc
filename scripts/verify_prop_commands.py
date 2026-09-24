"""Exact full native command consumer for selected manual engine controls."""
import json,random
from pathlib import Path
from component_assembly import f32
from fm_loader import normalize
from primary_controls import selected_properties,authority_ranges
from propulsion_model import prepare
from prop_commands import deliver
from prop_commands_native import PropCommandsNative

def main():
    m=PropCommandsNative();r=random.Random(195400);counts={};failures=[];hooks=set()
    def val(a,b):return f32(r.uniform(a,b))
    for n in ['yak-3','bf-109f-4']:
        fm=normalize(json.loads(Path('references/fm-2.59.0.13/'+n+'.blkx').read_text()));cp=selected_properties(fm);pp=prepare(fm)
        for i in range(1000):
            s=dict(engine=dict(throttle=1.,afterburner=False,mixture=.5,gear=0,regulator=val(.1,1.)),delivered=[val(-1,1) for _ in range(3)],trim_actual=[val(-1,1) for _ in range(3)],trim_requested=[val(-1,1) for _ in range(3)],command=val(0,1))
            snap=dict(commands=[val(-1,1) for _ in range(3)],throttle=val(.5,1.1),afterburner=bool(i%2),auto=bool(i%3),command=val(0,1),mixture=val(.1,1.2),gear=i%len(pp['engine']['stages']),radiator=val(0,1),oil_radiator=val(0,1))
            ranges=authority_ranges(cp,val(30,250));dt=f32(1/60);args=(cp,pp,snap,s,ranges,dt)
            a=m.delivery(*args);e=deliver(*args);counts[n]=counts.get(n,0)+1;hooks.update(m.calls)
            if a!=e:failures.append(dict(aircraft=n,case=i,diff={k:(a[k],e[k]) for k in e if a[k]!=e[k]}));break
        if failures:break
    report=dict(status='FAIL' if failures else 'PASS',binary_sha256=m.sha,counts=counts,substituted_calls=sorted(hooks),failures=failures,scope='Complete native101a4e5f0 normal return with connected inline engine/prop/transmission, full-real manual engine management. Prop auto/manual modes, throttle/WEP availability, 8-bit prop/radiator and .005 mixture quantization, selected compressor gear and regulator reset, primary commands and trim availability. No start/stop/feather/autopilot. Explicit held snapshot, not game-owner publication.')
    Path('analysis/prop-commands-validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
    if failures:raise SystemExit(1)
if __name__=='__main__':main()
