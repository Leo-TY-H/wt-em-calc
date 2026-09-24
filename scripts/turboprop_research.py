"""Turboprop turbine-kernel research, not a complete propulsion implementation.

The gas-generator speed is distinct from the transmission shaft speed. Native
1019eee00 obtains turbine RPM bounds from Compressor and shaft torque scale
from Main.Power/Main.RPMMax. The outer type-5 wrapper remains separate.
"""
import json
import random
from pathlib import Path
from component_assembly import f32,mul
from piston_model import div,RAD_PER_RPM
from jet_model import prepare as jet_properties,scalar_update
from jet_catalog import engines
from verify_jet_model import JetMachine,DATA
from verify_piston_loaded import load_engine_properties
from collision_native import HEAP

def properties(engine):
    main=engine['Main'];assert main['Type']=='TurboProp'
    p=jet_properties(main)
    p['bases'][2]=div(mul(f32(main['Power']),746.),mul(f32(main['RPMMax']),RAD_PER_RPM))
    p['max_omega']=mul(f32(engine['Compressor']['TurboChargerRPMMax']),RAD_PER_RPM)
    return p

class LoadedTurbine(JetMachine):
    def __init__(self,loader,address):
        super().__init__()
        self.loaded=bytes(loader.u.mem_read(address+0x578,0x1f8))
        self.u.mem_map(HEAP,0x2000000)
        self.u.mem_write(HEAP,bytes(loader.u.mem_read(HEAP,loader.cursor-HEAP)))
    def load(self,p):
        super().load(p)
        self.u.mem_write(DATA,self.loaded)

def main():
    rows=[];failures=[];rng=random.Random(0x19efd3b)
    def v(a,b):return f32(rng.uniform(a,b))
    for name in ['a2d','ia_58a_pucara','nt_tu_95m','quing_6','wyvern_s4']:
        fm=json.loads(Path('references/jet-catalog/fm/'+name+'.blkx').read_text())
        engine=next(e for _,e in engines(fm) if e['Main']['Type']=='TurboProp')
        loader,address=load_engine_properties(engine);p=properties(engine);m=LoadedTurbine(loader,address)
        expected=[p['bases'][0],p['bases'][1],p['bases'][2],p['bases'][3]]
        actual=loader.read(address+0x578+0x38,4)
        if actual!=expected:failures.append(dict(aircraft=name,base_diff=[actual,expected]))
        n=0
        for i in range(800):
            kw=dict(density=v(.2,1.4),speed=v(-50,300),omega=v(.25,1.15)*p['max_omega'],throttle=v(.1,1.1),
                    dt=f32(1/48),afterburner=bool(i%2),fuel_available=1000.,health=1.,shaft_fraction=v(.25,1.2),running=2)
            actual=m.scalar(p,**kw);expected=scalar_update(p,**kw);n+=1
            if actual!=expected:
                failures.append(dict(aircraft=name,case=i,diff={k:[actual[k],expected[k]] for k in expected if actual[k]!=expected[k]}));break
        rows.append(dict(aircraft=name,cases=n,base_torque_Nm=p['bases'][2],gas_generator_max_rad_s=p['max_omega'],main_shaft_max_rad_s=mul(engine['Main']['RPMMax'],RAD_PER_RPM)))
    report=dict(status='FAIL' if failures else 'PASS',binary_sha256=m.sha,rows=rows,failures=failures,
                scope='All five fixed-wing TurboProp FM records: original complete engine property loading followed by original scalar turbine consumer versus independent jet_model math with separately recovered turboprop torque and gas-generator normalization. Random healthy running kernel inputs. Does not validate type-5 outer wrapper, shaft/turbine coupling, transmission, coaxial propeller, nozzle aggregation, or whole-aircraft performance.')
    Path('analysis/turboprop-kernel-research.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
    if failures:raise SystemExit(1)

if __name__=='__main__':main()
