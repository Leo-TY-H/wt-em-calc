"""Differential checks of complete wing producer stages against aces bytes."""
import json, math, random, struct
from pathlib import Path
from unicorn import UC_HOOK_CODE
from unicorn.x86_const import *
from macho_scan import MachO
from component_assembly import f32,mul,add
from polar_model import make_polar,round_polar,pack_polar
from verify_component_stages import Stages
from verify_component_assembly import BASE,FRAME
from wing_model import selected_geometry,base_points,local_inputs,coefficients,postprocess_xy,evaluate,drag_terms
from verify_wing_stages import Wings

PARAM=BASE+0x11000


class WingMachine(Stages):
    def __init__(self):
        super().__init__();m=MachO()
        for va,size in [(0x106c5d000,0x3000),(0x106c64000,0x3000),
                        (0x10198c000,0x1000),(0x106e61000,0x1000),(0x100233000,0x1000)]:
            self.u.mem_map(va,size);self.u.mem_write(va,m.read(va,size))
        for va in [0x101a17630,0x101a17700]:
            if not any(a<=va<b for a,b,_ in self.u.mem_regions()):
                self.u.mem_map(va&~0xfff,0x1000)
        for va in [0x106e61c1b,0x106e614b9,0x100233d00,0x101a17630,0x101a17700]:
            self.u.hook_add(UC_HOOK_CODE,self.hook,begin=va,end=va)
        self.stop=None;self.radiator=0.;self.oil_radiator=0.
        for va in [0x106c5e353,0x106c5f67a,0x106c5f09d,0x106c6021a,0x106c606ca,
                   0x106c62226,0x106c62554,0x106c626a7,0x106c6049e,
                   0x106c5e01c,0x106c5e0af,0x106c5f089,0x106c601ed]:
            self.u.hook_add(UC_HOOK_CODE,self.stop_hook,begin=va,end=va)

    def stop_hook(self,u,address,size,data):
        if address==self.stop:u.emu_stop()

    def run(self,start,end,count):
        self.stop=end;self.u.emu_start(start,end,count=count)
        if self.u.reg_read(UC_X86_REG_RIP)!=end:raise RuntimeError('Wing block did not reach its stop')

    def double(self,addr,values):self.u.mem_write(addr,struct.pack('<'+'d'*len(values),*values))
    def qword(self,addr,x):self.u.mem_write(addr,struct.pack('<Q',x))
    def single(self,addr):return struct.unpack('<f',self.u.mem_read(addr,4))[0]

    def reset(self):
        super().reset();self.u.reg_write(UC_X86_REG_RSP,FRAME-0x1000)
        self.u.reg_write(UC_X86_REG_R14,PARAM)

    def hook(self,u,address,size,data):
        x=self.read_xmm(0)[0]
        if address==0x106e61c1b:self.xmm(0,[math.sin(x)])
        elif address==0x106e614b9:self.xmm(0,[math.atan2(x,self.read_xmm(1)[0])])
        elif address==0x100233d00:
            self.floats(u.reg_read(UC_X86_REG_RDI),[math.sin(x)])
            self.floats(u.reg_read(UC_X86_REG_RSI),[math.cos(x)])
        elif address==0x101a17630:self.xmm(0,[self.radiator])
        elif address==0x101a17700:self.xmm(0,[self.oil_radiator])
        sp=u.reg_read(UC_X86_REG_RSP)
        ret=struct.unpack('<Q',u.mem_read(sp,8))[0]
        u.reg_write(UC_X86_REG_RSP,sp+8);u.reg_write(UC_X86_REG_RIP,ret)

    def device_cd(self,fm,side,config,control_cd,kq):
        self.reset();self.radiator=config['radiator'];self.oil_radiator=config['oil_radiator']
        self.qword(BASE+0x6ed8,BASE+0x10000);self.qword(BASE+0x55a0,BASE+0x10000)
        bits=sum(int(v)<<(28+i) for i,v in enumerate([*config['gear_present'],config['central_gear_present']]))
        self.qword(BASE+0x1860,bits)
        for off,v in zip([0xc4,0x134,0x1a4],[*config['gear_animated'],config['central_gear_animated']]):
            self.u.mem_write(BASE+0x10000+off,bytes([v]))
        for off,key in [(0x2b50,'gear_position'),(0x2b58,'airbrake_position'),(0x2b64,'bomb_bay_position')]:
            self.floats(BASE+off,[config[key]])
        self.floats(BASE+0x18e4,[config['airbrake_health'][side]])
        self.floats(BASE+0x7ca8,[fm['Aerodynamics'].get(k,0.) for k in ['GearCd','GearCentralCd','AirbrakeCd','RadiatorCd','OilRadiatorCd','BombBayCd','FuseCd']])
        self.run(0x106c5df91,0x106c5e01c,500)
        self.run(0x106c5e0aa,0x106c5e0af,5)
        air=self.read_xmm(8)[0]
        self.floats(FRAME-(0x4e0 if side==0 else 0x500),[air])
        self.floats(FRAME-(0x660 if side==0 else 0x540),[control_cd])
        self.floats(FRAME-(0xb50 if side==0 else 0xaf0)+0x54,[kq])
        self.run(0x106c5eed9 if side==0 else 0x106c5ffa3,
                 0x106c5f089 if side==0 else 0x106c601ed,1000)
        result=self.read_xmm(0 if side==0 else 2)[0]
        self.radiator=0.;self.oil_radiator=0.
        return result

    def geometry(self,g,p,old,beta,flaps,gear,airbrake,pitch,modifier,health,damage):
        result=[]
        for i in range(2):
            self.reset()
            for off,values in [(0x8140,g['arm']),(0x8174,[g['sine_aos'],g['v_focus']]),
                               (0x8188,g['shifts']['FlapsShift']),
                               (0x8190,g['shifts']['AirbrakesShift']),
                               (0x8198,g['shifts']['GearShift']),
                               (0x81a0,g['shifts']['ElevonShift']),
                               (0x81a8,[g['aoa_shift']]),(0x1698,[pitch]),
                               (0x1678+4*i,[old[i]]),(0x18a4+16*i,health[i])]:self.floats(BASE+off,values)
            self.floats(PARAM+0x68,damage)
            self.double(FRAME-0x510,[modifier])
            self.floats(FRAME-0x568,[f32(math.sin(mul(g['dihedral'],f32(.01745329238474369))))])
            self.floats(FRAME-0x790,[mul(beta,f32(.01745329238474369))])
            self.floats(FRAME-0x6e0,[f32(math.sin(mul(beta,f32(.01745329238474369))))])
            off=0xb50 if i==0 else 0xaf0
            self.u.mem_write(FRAME-off,pack_polar(p[i]))
            self.qword(BASE+0x81c8,BASE+0x10000)
            self.u.mem_write(BASE+0x81d8,struct.pack('<I',len(g['aoa_shift_add'])))
            for j,(x,y) in enumerate(g['aoa_shift_add']):
                inv=f32(1/f32(g['aoa_shift_add'][j+1][0]-x)) if j+1<len(g['aoa_shift_add']) else 0.
                self.floats(BASE+0x10000+12*j,[x,inv,y])
            if i==0:
                self.u.reg_write(UC_X86_REG_RAX,1)
                self.xmm(0,[gear[i]]);self.xmm(4,[1.]);self.xmm(7,[flaps[i]]);self.xmm(8,[airbrake[i]])
                self.run(0x106c5e018,0x106c5e353,count=2000)
                result.append(self.read3(FRAME-0x650))
            else:
                self.xmm(0,[flaps[i]]);self.xmm(4,[1.]);self.xmm(12,[airbrake[i]]);self.xmm(13,[gear[i]])
                self.run(0x106c5f37d,0x106c5f67a,count=2000)
                result.append(self.read3(FRAME-0x640))
        return result

    def wing(self,i,p,position,cog,velocity,omega,disturbance,dihedral,control,
             incidence,flap,health,body_aoa,convert,cd_factor,extra_cd,rho,engine_x=0.,engine_swirl=0.):
        self.reset()
        self.double(BASE+0x1618,velocity);self.double(BASE+0x15e0,omega)
        self.floats(BASE+0x5320,cog);self.floats(BASE+0x813c,[incidence])
        self.floats(BASE+0x8454,[body_aoa]);self.u.mem_write(BASE+0x7c0a,bytes([convert]))
        self.floats(BASE+0x18b0+16*i,[health])
        self.floats(FRAME-0x7a0,[rho]);self.floats(FRAME-0x4c0,[mul(rho,.5)])
        self.floats(FRAME-0x56c,[f32(math.cos(mul(dihedral,f32(.01745329238474369)))),f32(math.sin(mul(dihedral,f32(.01745329238474369))))])
        self.qword(BASE+0x6ed8,BASE+0x10000);self.qword(BASE+0x55a0,BASE+0x10000)
        self.floats(BASE+0x7cc0,[extra_cd]);self.floats(BASE+0x7c8c,[0.,1.,0.,1.])
        self.double(FRAME-0x540,[engine_x]);self.double(FRAME-0x4f0,[engine_swirl])
        self.floats(FRAME-0x4a0,[cd_factor]);self.double(BASE+0x5348,[50000.])
        self.floats(FRAME-0x8f0,[1.]);self.floats(FRAME-0x880,[1.])
        if i==0:
            self.u.mem_write(FRAME-0xb50,pack_polar(p));self.floats(FRAME-0x650,position)
            self.floats(FRAME-0xa90,control);self.floats(FRAME-0x490,[flap])
            self.floats(FRAME-0x4c0,disturbance[:2]);self.floats(FRAME-0x480,disturbance[2:])
            self.run(0x106c5e8fc,0x106c5f09d,count=10000)
            return dict(flow=self.read3(FRAME-0xa40),aoa=self.single(FRAME-0x670),
                        reference_aoa=self.single(FRAME-0x4b0),q=self.single(FRAME-0x480),
                        cx=f32(self.single(FRAME-0x6b0)+self.single(FRAME-0x4e0)),
                        cy=self.read3(FRAME-0x550)[1],cy_reference=self.read3(FRAME-0x5f0)[1],
                        cy_add=self.single(FRAME-0x580),raw_cl=self.read_xmm(0)[0])
        self.u.mem_write(FRAME-0xaf0,pack_polar(p));self.floats(FRAME-0x640,position)
        self.floats(FRAME-0xa7c,control);self.floats(FRAME-0x5d0,[flap])
        self.floats(FRAME-0x560,disturbance[:2]);self.floats(FRAME-0x490,disturbance[2:])
        self.double(FRAME-0x530,[engine_swirl*.07000000029802322])
        self.run(0x106c5f9c9,0x106c6021a,count=10000)
        return dict(flow=self.read3(FRAME-0xa20),aoa=self.single(FRAME-0x6d0),
                    q=self.single(FRAME-0x490),cx=self.single(FRAME-0x500),
                    cy=self.single(FRAME-0x510),cy_reference=self.single(FRAME-0x4f0),
                    cy_add=self.single(FRAME-0x590),raw_cl=self.read_xmm(0)[0])

    def corrections(self,xy,aoa,crit,cy_add,q,body_q,dt,height,length,ordering,spin,increment,cap,
                    rudder_health,yaw,yawrate,use_spin,spinrate,loss):
        self.reset()
        for off,v in [(0x5f0,xy[0]),(0x550,xy[1]),(0x670,[aoa[0]]),(0x6d0,[aoa[1]]),
                      (0x4e0,[mul(f32(crit[0]+crit[1]),.5)]),(0x500,[f32(crit[0]+crit[1])]),
                      (0x580,[cy_add[0]]),(0x590,[cy_add[1]]),(0x480,[q[0]]),(0x490,[q[1]]),
                      (0x5b0,[body_q]),(0x5f8,[increment]),(0x5c0,[dt])]:self.floats(FRAME-off,v)
        self.double(FRAME-0x4d0,[max(height,0.)]);self.floats(BASE+0x7530,[length])
        self.floats(BASE+0x1690,[spin]);self.floats(BASE+0x169c,[yaw]);self.double(BASE+0x15e8,[yawrate])
        self.floats(BASE+0x18d8,[rudder_health]);self.floats(BASE+0x8494,[spinrate]);self.floats(BASE+0x8180,loss)
        self.u.mem_write(BASE+0x817c,bytes([use_spin]));self.xmm(0,[cap])
        self.run(0x106c605a1 if ordering else 0x106c64e16,0x106c606ca,count=1500)
        return dict(xy=[self.read3(FRAME-0x5f0)[:2],self.read3(FRAME-0x550)[:2]],
                    spin=self.single(BASE+0x1690),gyroscopic_scale=struct.unpack('<d',self.u.mem_read(FRAME-0xb80,8))[0],
                    vertical_area_scale=self.single(FRAME-0x9d0),vertical_control_scale=self.single(FRAME-0x850))

    def vectors(self,xy,drag,lift,areas,sv,sb):
        self.reset()
        for off,v in [(0x6b0,[lift[0]]),(0x540,[drag[0]]),(0x6e0,[drag[1]]),
                      (0x950,[sv]),(0x940,[sb]),(0x8f0,[areas[0]]),(0x880,[areas[1]]),
                      (0x5f0,xy[0]),(0x550,xy[1])]:self.floats(FRAME-off,v)
        self.xmm(9,[lift[1]])
        self.run(0x106c621d3,0x106c62226,count=100)
        self.run(0x106c62528,0x106c62554,count=100)
        self.run(0x106c62643,0x106c626a7,count=100)
        return [self.read3(FRAME-0x738),self.read3(FRAME-0x728)]

    def composed(self,g,p,v,w,cog,controls,rho,dt,old,beta,body_aoa,inertia,height,ordering):
        from wing_stages import effective_areas
        from wing_model import div,RAD
        from component_assembly import add
        health=[[1.]*4 for _ in range(2)];z=[0.,0.]
        base=self.geometry(g,p,old,beta,z,z,z,0.,0.,health,z)
        c=[self.wing(i,p[i],base[i],cog,v,w,[0.]*3,g['dihedral'],controls[i],g['incidence'],0.,1.,body_aoa,False,1.,0.,rho) for i in range(2)]
        area=effective_areas(g['areas'],health);q=[r['q'] for r in c]
        cv=f32(math.cos(mul(g['dihedral'],RAD)));sv=f32(math.sin(mul(g['dihedral'],RAD)))
        sb=f32(math.sin(mul(beta,RAD)));cb=abs(f32(math.cos(mul(beta,RAD))))
        weight=[div(mul(mul(mul(q[i],area[i]),cv),base[i][2]),f32(inertia)) for i in range(2)]
        self.stop=0x106c6049e
        points=Wings.points(self,[r['cx'] for r in c],[r['cy'] for r in c],
                            [r['cy_reference'] for r in c],[r['cy_add'] for r in c],weight,dt,w[0],g['area'],g['span'],base,
                            [[r['clToCm0'],r['clToCm1']] for r in p])
        drag=[mul(-q[i],c[i]['cx']) for i in range(2)];lift=[mul(q[i],points['cy'][i]) for i in range(2)]
        xy=[[mul(drag[i],cb),mul(lift[i],cv)] for i in range(2)]
        vv=list(map(f32,v));qb=mul(add(add(mul(vv[0],vv[0]),mul(vv[1],vv[1])),mul(vv[2],vv[2])),mul(rho,.5))
        post=self.corrections(xy,[r['aoa'] for r in c],[r['aoaCritH'] for r in p],[r['cy_add'] for r in c],q,qb,dt,height,10.,ordering,0.,f32(.2),0.,1.,0.,w[1],g['use_spin_loss'],height,g['spin_loss'])
        return dict(points=points['points'],forces=self.vectors(post['xy'],drag,lift,area,sv,sb),postprocess=post)


def main():
    m=WingMachine();rng=random.Random(39001611);failures=[];counts={};worst={}
    def check(kind,a,b,exact=True):
        counts[kind]=counts.get(kind,0)+1
        def leaves(x):
            if isinstance(x,dict):return [v for k in sorted(x) for v in leaves(x[k])]
            if isinstance(x,(list,tuple)):return [v for y in x for v in leaves(y)]
            return [x]
        aa,bb=leaves(a),leaves(b)
        err=max(abs(x-y) for x,y in zip(aa,bb));worst[kind]=max(worst.get(kind,0.),err)
        if len(aa)!=len(bb) or not all(x==y if exact else math.isclose(x,y,rel_tol=3e-5,abs_tol=3e-5) for x,y in zip(aa,bb)):
            failures.append(dict(kind=kind,actual=a,expected=b,max_error=err))
    def vals(n,a,b):return [f32(rng.uniform(a,b)) for _ in range(n)]
    for j in range(1000):
        fm={'Aerodynamics':dict(zip(['GearCd','GearCentralCd','AirbrakeCd','RadiatorCd','OilRadiatorCd','BombBayCd','FuseCd'],vals(7,0,.1)))}
        cfg=dict(gear_position=vals(1,-.2,1.2)[0],airbrake_position=vals(1,-.2,1.2)[0],bomb_bay_position=vals(1,-.2,1.2)[0],
                 airbrake_health=vals(2,0,1),gear_present=[bool(rng.randrange(2)) for _ in range(2)],central_gear_present=bool(rng.randrange(2)),
                 gear_animated=[bool(rng.randrange(2)) for _ in range(2)],central_gear_animated=bool(rng.randrange(2)),
                 radiator=vals(1,0,1)[0],oil_radiator=vals(1,0,1)[0])
        terms=drag_terms(fm,**cfg);side=j%2;e=terms[side];control=vals(1,0,.1)[0];kq=vals(1,.5,1.5)[0]
        expected=mul(add(add(e['airbrake'],add(add(add(e['gear'],control),add(e['oil_radiator'],add(e['radiator'],e['central_gear']))),e['fuselage'])),e['bomb_bay']),kq)
        check('device_cd',m.device_cd(fm,side,cfg,control,kq),expected)
    for name in ['f_16a_block_15_adf','saab_jas39c']:
        fm=json.loads(Path('references/fm-2.59.0.13/'+name+'.blkx').read_text());g=selected_geometry(fm)
        for j in range(200):
            p=[round_polar(make_polar(fm['Aerodynamics']['WingPlane']['FlapsPolar0'],g['span'],g['area'],rng.uniform(0,1.8))) for _ in range(2)]
            old=vals(2,-50,90);beta=vals(1,-30,30)[0];flaps=vals(2,0,1);gear=vals(2,0,1);air=vals(2,0,1)
            pitch=vals(1,-1,1)[0];modifier=vals(1,-.2,.2)[0];health=[vals(4,.1,1) for _ in range(2)];damage=vals(2,0,1)
            args=[g,p,old,beta,flaps,gear,air,pitch,modifier,health,damage]
            actual=m.geometry(*args);expected=base_points(*args)
            check('geometry',actual,expected)
            for side in range(2):
                v=vals(3,-300,300);w=vals(3,-3,3);cog=vals(3,-1,1);d=vals(3,-10,10)
                control=vals(5,-.2,.2);inc=g['incidence'];h=health[side][3]
                bodyaoa=vals(1,-60,60)[0];convert=bool(j%2);cd=vals(1,.8,1.5)[0];extra=vals(1,0,.1)[0]
                rho=vals(1,.2,1.3)[0];engine_x=vals(1,0,30)[0];swirl=vals(1,-10,10)[0]
                dihedral=g['dihedral'] if j%2 else vals(1,-10,10)[0]
                flow=local_inputs(side,expected[side],cog,v,w,d,dihedral,engine_x,swirl)
                coeff=coefficients(p[side],flow,control,inc,flaps[side],h,bodyaoa,convert,cd,extra)
                a=m.wing(side,p[side],expected[side],cog,v,w,d,dihedral,control,inc,flaps[side],h,bodyaoa,convert,cd,extra,rho,engine_x,swirl)
                a.pop('reference_aoa',None)
                b={k:coeff[k] for k in ['aoa','cx','cy','cy_reference','cy_add','raw_cl']};b.update(flow=flow['flow'],q=mul(flow['speed_squared'],mul(rho,.5)))
                check('local_coefficients',a,b)
    for j in range(800):
        args=[[vals(2,-5e4,5e4) for _ in range(2)],vals(2,-30,75),vals(2,5,35),vals(2,-.2,.2),vals(2,1,6e4),
              *vals(1,1,6e4),*vals(1,.001,.1),*vals(1,-10,50),*vals(1,.1,30),bool(j%2),
              *vals(1,0,1.5),f32(.2),*vals(1,0,1),*vals(1,0,1),*vals(1,-1,1),*vals(1,-2,2),bool(j%3),
              *vals(1,0,20),vals(2,0,.3)]
        check('corrections',m.corrections(*args),postprocess_xy(*args))
    for name in ['f_16a_block_15_adf','saab_jas39c']:
        fm=json.loads(Path('references/fm-2.59.0.13/'+name+'.blkx').read_text());g=selected_geometry(fm)
        for j in range(100):
            mach=rng.uniform(.2,1.8)
            p=[round_polar(make_polar(fm['Aerodynamics']['WingPlane']['FlapsPolar0'],g['span'],g['area'],mach)) for _ in range(2)]
            speed=rng.uniform(100,500);alpha=rng.uniform(-10,45);beta=f32(rng.uniform(-8,8))
            v=[speed*math.cos(math.radians(alpha)), -speed*math.sin(math.radians(alpha)), -speed*math.sin(math.radians(beta))]
            w=vals(3,-.2,.2);cog=vals(3,-.3,.3);controls=[vals(5,-.1,.1) for _ in range(2)]
            rho=f32(rng.uniform(.2,1.225));dt=f32(1/60);old=[f32(alpha)]*2;inertia=50000.;height=f32(rng.uniform(0,30));order=bool(j%2)
            a=m.composed(g,p,v,w,cog,controls,rho,dt,old,beta,f32(alpha),inertia,height,order)
            b=evaluate(g,p,v,w,cog,controls,rho,dt,old,beta,f32(alpha),inertia_x=inertia,height=height,ground_length=10.,ordering=order,spin_agl=height)
            b={k:b[k] for k in a}
            check('composed_forces_points',a,b)
    report=dict(binary_sha256=m.sha,counts=counts,evaluations=sum(counts.values()),machine_block_executions=3*counts['device_cd']+2*counts['geometry']+counts['local_coefficients']+counts['corrections']+9*counts['composed_forces_points'],max_absolute_error=worst,failures=failures,
                limitations='Geometry checks start with runtime polar and statically adapted wing-loader data. Local coefficient blocks execute native polar kernels and host float atan2/sin/sincos. Corrections include both ordering and stall/spin branches. Prepared controls are supplied; complete timestep/rigid body not executed.')
    Path('analysis/wing-model-validation.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='failures'},indent=2));print('FAILURES',len(failures));print(json.dumps(failures[:5],indent=2))
    if failures:raise SystemExit(1)


if __name__=='__main__':main()
