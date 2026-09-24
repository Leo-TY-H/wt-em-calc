"""Execute original prepared-wing interpolation and sweep-control instructions."""
import copy,json,random,struct
from pathlib import Path
from unicorn.x86_const import *
from component_assembly import f32
from macho_scan import MachO
from polar_runtime import PolarMachine,DATA,STACK,STOP,pack_runtime,unpack_runtime
from wing_sweep import prepare,blend,schedule,available


class SweepMachine(PolarMachine):
    def __init__(self):
        super().__init__();m=MachO()
        for va,size in [(0x1019e1000,0x1000),(0x107615000,0x1000)]:
            self.u.mem_map(va,size);self.u.mem_write(va,m.read(va,size))
        self.u.mem_write(0x107615358,struct.pack('<Q',STOP+0x100))

    def pack(self,p,address,output=False):
        g=p['geometry'];s=g['strength'];a=g['areas'];d=bytearray(0xd8)
        struct.pack_into('<17f',d,0,g['span'],g['incidence'],*g['arm'],g['sweep'],g['taper'],g['dihedral'],
                         *a[0][:3],*a[1][:3],a[0][3],g['sine_aos'],g['v_focus'])
        d[68]=g['use_spin_loss']
        struct.pack_into('<11f',d,72,*g['spin_loss'],
                         *(v for k in ['FlapsShift','AirbrakesShift','GearShift','ElevonShift'] for v in g['shifts'][k]),g['aoa_shift'])
        struct.pack_into('<I5f',d,116,g['downwash_type'],g['downwash_coefficient'],*s['force'],s['ias'],s['mach'])
        for offset,points in [(144,g['aoa_shift_add']),(168,p['polars'])]:
            base=address+(0x100 if offset==144 else 0x400)
            struct.pack_into('<Q',d,offset,base);struct.pack_into('<II',d,offset+16,0 if output else len(points),32)
            for i,(x,y) in enumerate(points):
                inv=f32(1/f32(points[i+1][0]-x)) if i+1<len(points) else 0.
                self.u.mem_write(base+12*i,struct.pack('<ff',x,inv)+struct.pack('<f' if offset==144 else '<I',y if offset==144 else i))
        struct.pack_into('<Q',d,192,address+0x800);struct.pack_into('<II',d,208,len(p['polars']),len(p['polars']))
        for i,(_,r) in enumerate(p['polars']):
            dest=address+0x800+0x1e0*i;self.u.mem_write(dest,pack_runtime(r,dest))
        self.u.mem_write(address,bytes(d))

    def call(self,a,b,k):
        x,y,out=DATA,DATA+0x3000,DATA+0x6000
        for address,p in [(x,a),(y,b),(out,a)]:self.pack(p,address,output=address==out)
        self.u.mem_write(STACK+0xfff8,struct.pack('<Q',STOP));self.u.reg_write(UC_X86_REG_RSP,STACK+0xfff8)
        for reg,value in [(UC_X86_REG_RDI,x),(UC_X86_REG_RSI,y),(UC_X86_REG_RDX,out)]:self.u.reg_write(reg,value)
        self.u.reg_write(UC_X86_REG_XMM0,int.from_bytes(struct.pack('<f',k),'little'))
        self.u.emu_start(0x1019e1670,STOP,count=200000)
        if self.u.reg_read(UC_X86_REG_RIP)!=STOP:raise RuntimeError('Sweep interpolation did not return')
        return self.snapshot(out)

    def snapshot(self,address):
        d=bytes(self.u.mem_read(address,0xd8));out=dict(scalars=list(struct.unpack_from('<17f',d)),
            spin=bool(d[68]),shifts=list(struct.unpack_from('<11f',d,72)),
            downwash=struct.unpack_from('<I',d,116)[0],limits=list(struct.unpack_from('<5f',d,120)))
        for key,offset in [('aoa',144),('flap_knots',168)]:
            base=struct.unpack_from('<Q',d,offset)[0];count=struct.unpack_from('<I',d,offset+16)[0]
            out[key]=[list(struct.unpack('<fff' if key=='aoa' else '<ffI',self.u.mem_read(base+12*i,12))) for i in range(count)]
        base=struct.unpack_from('<Q',d,192)[0];count=struct.unpack_from('<I',d,208)[0]
        out['polars']=[unpack_runtime(bytes(self.u.mem_read(base+0x1e0*i,0x1e0)),base+0x1e0*i) for i in range(count)]
        return out


def main():
    m=SweepMachine();rng=random.Random(140023);failures=[];cases=0;families={}
    for path in sorted(Path('references/jet-catalog/fm').glob('*.blkx')):
        fm=json.loads(path.read_text())
        if not any(k.startswith('WingPlaneSweep') for k in fm.get('Aerodynamics',{})):continue
        family=prepare(fm);families[path.stem]=len(family)
        for (_,a),(_,b) in zip(family,family[1:]):
            for k in [0.,.001,.2,.49999997,.5,.8,1.]+[rng.random() for _ in range(5)]:
                actual=m.call(a,b,f32(k));expected=blend(a,b,k);m.pack(expected,DATA+0x9000)
                expected=m.snapshot(DATA+0x9000);cases+=1
                if actual!=expected:failures.append(dict(aircraft=path.stem,k=k,fields=[key for key in actual if actual[key]!=expected[key]]))
    # Nonmatching scalar, flap and Cm grids exercise the inner interpolation.
    fm=json.loads(Path('references/jet-catalog/fm/f_14a_early.blkx').read_text());family=prepare(fm)
    a=copy.deepcopy(family[0][1]);b=copy.deepcopy(family[-1][1])
    a['geometry']['aoa_shift_add']=[list(map(f32,p)) for p in [[-5.,.27],[0.,-.13],[3.,.35],[9.,.75]]]
    b['geometry']['aoa_shift_add']=[list(map(f32,p)) for p in [[-4.,.81],[1.,-.71],[7.,.03]]]
    a['polars'].insert(1,(f32(.37),copy.deepcopy(a['polars'][0][1])))
    for k in [0.,.23,.5,.83,1.]:
        actual=m.call(a,b,f32(k));m.pack(blend(a,b,k),DATA+0x9000);expected=m.snapshot(DATA+0x9000);cases+=1
        if actual!=expected:failures.append(dict(aircraft='synthetic mismatched grids',k=k,fields=[key for key in actual if actual[key]!=expected[key]]))
    report=dict(binary_sha256=m.sha,aircraft=len(families),families=families,cases=cases,failures=failures,
                comparison='Exact float32 geometry, strength, tables and full prepared polars from native 1019e1670',
                limitations='Prepared input structs; this does not execute the original BLK loader or prove sweep optimization.')
    Path('analysis/wing-sweep-validation.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='families'},indent=2))
    if failures:raise SystemExit(1)


if __name__=='__main__':main()
