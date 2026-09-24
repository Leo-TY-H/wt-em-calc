"""Actual full snapshot delivery for every installed fixed-wing prop graph."""
import argparse,json
from pathlib import Path
from propulsion_general_native import PropulsionGeneralNative
from prop_steady import initial_state
from component_assembly import f32

ROOT=Path(__file__).resolve().parents[1]


def main():
    ap=argparse.ArgumentParser();ap.add_argument('names',nargs='*');ap.add_argument('--automatic',action='store_true');ap.add_argument('--report',default='analysis/prop-integration/control-delivery-validation.json');args=ap.parse_args()
    names=args.names or [r['aircraft'] for r in json.loads((ROOT/'references/prop-native-config.json').read_text())['aircraft']]
    failures=[];records=[]
    for name in names:
        m=PropulsionGeneralNative();fm=json.loads((ROOT/'references/jet-catalog/fm'/(name+'.blkx')).read_text());m.configure(fm);p=m.config
        for command in ([255] if args.automatic else [0,127,254,255]):
            commands=[command if r['manual'] else 255 for r in p['propellers']]
            auto=[True if args.automatic else r['automatic'] and not r['manual'] for r in p['propellers']]
            gears=[len(e['properties']['stages'])-1 if e['properties']['manual_compressor'] else 0 for e in p['engines']]
            state=initial_state(p,[100.,0.,0.],3000.,commands=commands,automatic=auto,gears=gears,nitro=10.,engine_control_mode='automatic' if args.automatic else 'optimized')
            if args.automatic:
                m.step(state,prepare_only=True,nitro=10.)
                for a in m.engine_addresses:
                    m.run(0x1019f85b0,[a])
                    assert m.read(a+0xac,1)[0]==1. and m.u.mem_read(a+0xcf,1)[0]==1
            m.step(state,prepare_only=True,nitro=10.)
            if args.automatic:
                for a in m.engine_addresses:m.u.mem_write(a+0xcf,b'\0')
            m.uint(m.fm+0x8528,len(p['engines']));m.uint(m.fm+0x2f58,len(p['engines']))
            m.u.mem_write(m.fm+0x3658,b'\x06');m.qword(m.fm+0x6ef0,m.seed+0x280)
            m.floats(m.fm+0x7c0c,[1.,1.,1.]);m.floats(m.seed+0x100,[-1.,1.,-1.,1.,-1.,1.])
            for i,e in enumerate(p['engines']):
                t=next((t for t in p['transmissions'] if any(l['index']==i for l in t['engines'])),None)
                pi=t['propellers'][0]['index'] if t else None
                automatic=auto[pi] if pi is not None else False;byte=commands[pi] if pi is not None else 255
                base=m.fm+0x2f5c+0x3c*i;m.u.mem_write(base,bytes([32+int(automatic)+(24 if args.automatic else 0)]))
                m.floats(base+12,[state['engines'][i]['throttle'],min(1.,(byte+.5)/255.),0.,0.])
                m.uint(base+28,3);m.uint(base+32,gears[i])
                mixture=state['engines'][i]['mixture'];index=round(mixture/.005)
                m.floats(base+36,[(index+.5)/200.,1.]);m.u.mem_write(base+44,b'\1')
            m.xmm(0,[f32(1/48)]);m.run(0x101a4e5f0,[m.fm,m.seed,m.seed+0x100,71,m.seed+0x200])
            diff=[]
            for i,a in enumerate(m.engine_addresses):
                expected=state['engines'][i]
                if bool(m.u.mem_read(a+0xcf,1)[0])!=(args.automatic and any(any(l['index']==i for l in t['engines']) for t in p['transmissions'])):diff.append(dict(engine=i,automatic_mixture='flag mismatch'))
                actual=dict(throttle=m.read(a+0xa4,1)[0],mixture=m.read(a+0xac,1)[0],gear=m.read(a+0xb4,1,'I')[0],
                            radiator=m.read(a+0x9c,1)[0],oil_radiator=m.read(a+0xa0,1)[0])
                wanted={k:expected[k] for k in ['throttle','mixture','gear']};wanted.update(radiator=0.,oil_radiator=0.)
                if actual!=wanted:diff.append(dict(engine=i,actual=actual,expected=wanted))
            for i,a in enumerate(m.prop_addresses):
                actual=[m.read(a+0x48,1)[0],bool(m.u.mem_read(a+0x4d,1)[0])]
                expected=[state['propellers'][i]['command'],state['propellers'][i]['auto']]
                if actual!=expected:diff.append(dict(propeller=i,actual=actual,expected=expected))
            if diff:failures.append(dict(aircraft=name,command=command,differences=diff))
        records.append(name);print(name,'FAIL' if failures and failures[-1]['aircraft']==name else 'PASS',flush=True)
    report=dict(status='FAIL' if failures else 'PASS',aircraft=records,cases=len(records)*(1 if args.automatic else 4),automatic=args.automatic,failures=failures,binary_sha256=m.sha,
        scope='Original 101a4e5f0 snapshot consumer on actual installed graph; closed radiator override. Automatic mode delivers propeller/mixture/turbo flags and snapshot compressor-auto bit. Manual mode checks four pitch bytes and selected stage/mixture. Healthy full-real branch; no network scheduler or live controller claim.')
    (ROOT/args.report).write_text(json.dumps(report,indent=2)+'\n');print(report['status'],len(failures),flush=True)
    if failures:raise SystemExit(1)


if __name__=='__main__':main()
