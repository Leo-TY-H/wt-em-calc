"""Machine checks for complete type-2 wake and secondary angle/history stages."""
import json,math,random,struct
from pathlib import Path
from unicorn import UC_HOOK_CODE
from unicorn.x86_const import *
from component_assembly import f32
from tail_model import type2_downwash,history_increment,wake_projection
from verify_downwash import Wake
from verify_component_assembly import BASE,FRAME


class TailMachine(Wake):
    def __init__(self):
        super().__init__()
        for address in [0x106e61c1b,0x106e614b9,0x106e6164b]:
            self.u.hook_add(UC_HOOK_CODE,self.tail_hook,begin=address,end=address)

    def tail_hook(self,u,address,size,data):
        a,b=self.read_xmm(0)[0],self.read_xmm(1)[0]
        value=math.sin(a) if address==0x106e61c1b else math.atan2(a,b) if address==0x106e614b9 else math.fmod(a,b)
        self.xmm(0,[f32(value)])
        sp=u.reg_read(UC_X86_REG_RSP);ret=struct.unpack('<Q',u.mem_read(sp,8))[0]
        u.reg_write(UC_X86_REG_RSP,sp+8);u.reg_write(UC_X86_REG_RIP,ret)

    def double_xmm(self,n,v):
        self.u.reg_write(UC_X86_REG_XMM0+n,int.from_bytes(struct.pack('<2d',*(list(v)+[0.,0.])[:2]),'little'))

    def wake(self,span,area,sweep,taper,dihedral,wing_points,tail_points,velocity,omega_x,alpha_degrees,beta_degrees,current_cl,previous_cl,dt,coefficient,tail_flows,travel_cap=1.):
        self.reset();u=self.u
        u.reg_write(UC_X86_REG_RSP,FRAME-0x3000)
        u.mem_write(BASE+0x55a0,struct.pack('<Q',BASE+0x4000))
        u.mem_write(BASE+0x15e0,struct.pack('<d',omega_x))
        for off,value in [(0x8138,span),(0x8254,area),(0x814c,sweep),(0x8150,taper),(0x8154,dihedral),(0x81b0,coefficient)]:self.floats(BASE+off,[value])
        self.floats(BASE+0x1688,previous_cl);self.floats(0x107d6fc54,[travel_cap])
        for off,value in [(0x790,f32(beta_degrees*f32(.01745329238474369))),(0x8e0,f32(alpha_degrees*f32(.01745329238474369))),(0x5c0,dt),(0x4b0,current_cl[0]),(0x4a0,current_cl[1])]:self.floats(FRAME-off,[value])
        for off,v in [(0x650,wing_points[0]),(0x640,wing_points[1]),(0x7f0,tail_points[0]),(0x7e0,tail_points[1]),(0x778,tail_flows[0]),(0x630,tail_flows[1])]:self.floats(FRAME-off,v)
        self.double_xmm(2,[velocity[0]]);self.double_xmm(3,velocity[1:])
        u.emu_start(0x106c60fa0,0x106c618d9,count=10000)
        angle=list(struct.unpack('<2f',u.mem_read(FRAME-0x4a0,8)))
        return dict(angles=angle[::-1],previous_cl=list(struct.unpack('<2f',u.mem_read(BASE+0x1688,8))))

    def history(self,current,previous,scale,inertia,dt,vertical=False):
        self.reset();u=self.u;u.reg_write(UC_X86_REG_RSP,FRAME-0x3000)
        if not vertical:
            self.floats(BASE+0x8454,[current]);self.floats(BASE+0x1680,[previous]);self.floats(FRAME-0x828,[scale]);self.floats(BASE+0x6f34,[inertia]);self.floats(FRAME-0x5c0,[dt])
            u.emu_start(0x106c619c3,0x106c61a62,count=1000)
            return self.read_xmm(7)[0]
        self.floats(BASE+0x8458,[current]);self.floats(BASE+0x1684,[previous]);self.floats(FRAME-0x6c0,[scale]);self.floats(BASE+0x713c,[inertia]);self.floats(FRAME-0x5c0,[dt])
        u.emu_start(0x106c61f42,0x106c62019,count=1000)
        return self.read_xmm(15)[0]


def main():
    m=TailMachine();rng=random.Random(493920);failures=[];maxerr=0;counts=dict(wake=0,horizontal_history=0,vertical_history=0)
    def r(a,b):return f32(rng.uniform(a,b))
    for i in range(1000):
        wp=[[r(-2,2),r(-2,2),r(1,5)],[r(-2,2),r(-2,2),r(-5,-1)]]
        tp=[[r(-8,8),r(-2,2),r(1,5)],[r(-8,8),r(-2,2),r(-5,-1)]]
        v=[rng.uniform(1,600),rng.uniform(-50,50),rng.uniform(-30,30)]
        flows=[[r(1,600),r(-100,100),r(-30,30)] for _ in range(2)]
        args=(r(6,18),r(10,100),r(0,75),r(1,9),r(-20,20),wp,tp,v,rng.uniform(-3,3),r(-80,80),r(-60,60),[r(-3,3),r(-3,3)],[r(-3,3),r(-3,3)],r(.005,.05),[0.,f32(.8),1.][i%3],flows,r(.005,1.))
        expected=type2_downwash(*args);actual=m.wake(*args);counts['wake']+=1
        error=max(abs(a-b) for a,b in zip(actual['angles'],expected['angles']));maxerr=max(maxerr,error)
        if actual['angles']!=expected['angles'] or actual['previous_cl']!=expected['previous_cl']:
            failures.append(dict(case=i,stage='wake',actual=actual,expected=expected,args=args))
        for vertical in [False,True]:
            args=(r(-180,180),r(-180,180),r(.1,3),r(-.5,.5),r(.005,.05),vertical)
            actual=m.history(*args);expected=history_increment(*args);counts['vertical_history' if vertical else 'horizontal_history']+=1
            if actual!=expected:failures.append(dict(case=i,stage='history',args=args,actual=actual,expected=expected))
    report=dict(counts=counts,slice_executions=sum(counts.values()),binary_sha256=m.sha,max_absolute_wake_error=maxerr,failures=failures,limitations='Prepared finite geometry, aircraft body angles, wing CL and component flow. No propeller wash. exp/sincos/sin/atan2/fmod imports use host math rounded to float32. Quaternion and wake arithmetic run unchanged.')
    Path('analysis/tail-model-validation.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k!='failures'},indent=2));print('FAILURES',len(failures));print(json.dumps(failures[:2],indent=2))
    if failures:raise SystemExit(1)




class SecondaryMachine(TailMachine):
    def __init__(self):
        super().__init__()
        from macho_scan import MachO
        from verify_polar_machine_code import Kernel
        m=MachO();self.u.mem_map(0x10198c000,0x1000);self.u.mem_write(0x10198c000,m.read(0x10198c000,0x1000));self.kernel=Kernel(m)

    def coeff_eval(self,p,a,b,cl,cd):
        return self.kernel.call(0x10198c320,p,[a,b,cl,cd])

    def prepared(self,polars,controls,local_angles,body_angles,previous_angles,incidence,flow_inertia,dt,areas,health,convert_aoa=False,horizontal_bias=0.,history_scales=(1.,1.),vertical_control_scale=1.,vertical_area_add=0.):
        from polar_model import pack_polar
        self.reset();u=self.u;u.reg_write(UC_X86_REG_RSP,FRAME-0x3000)
        u.mem_write(BASE+0x7c0a,bytes([convert_aoa]))
        self.floats(BASE+0x8454,body_angles);self.floats(BASE+0x1680,previous_angles)
        for off,value in [(0x6f1c,incidence['hstab']),(0x7124,incidence['vstab']),(0x6f34,flow_inertia['hstab']),(0x713c,flow_inertia['vstab']),(0x7344,flow_inertia['fuselage'])]:self.floats(BASE+off,[value])
        for key,off in dict(fuselage=0x83fc,left_main=0x8420,left_elevator=0x8424,right_main=0x8428,right_elevator=0x842c,v_main=0x8430,rudder=0x8434).items():self.floats(BASE+off,[areas[key]])
        for key,off in dict(fuselage=0x18a0,left_main=0x18c4,left_elevator=0x18c8,right_main=0x18cc,right_elevator=0x18d0,v_main=0x18d4,rudder=0x18d8).items():self.floats(BASE+off,[health[key]])
        for off,value in [(0x5c0,dt),(0x998,horizontal_bias),(0x828,history_scales[0]),(0x6c0,history_scales[1]),(0x850,vertical_control_scale),(0x5f8,vertical_area_add)]:self.floats(FRAME-off,[value])
        for name,off in [('hstab',0xc58),('vstab',0x3f0),('fuselage',0x470)]:u.mem_write(FRAME-off,pack_polar(polars[name]))
        for name,off in [('left_hstab',0xb68),('right_hstab',0xa68),('vstab',0x9b4)]:self.floats(FRAME-off,controls[name])
        self.floats(FRAME-0x4a0,[local_angles[1],local_angles[0]])
        u.reg_write(UC_X86_REG_R13,FRAME-0xb68);u.reg_write(UC_X86_REG_R15,FRAME-0xa68)
        u.emu_start(0x106c6191a,0x106c61ebf,count=10000)
        result={n:[self.read3(FRAME-a)[0],self.read3(FRAME-b)[0]] for n,a,b in [('left_hstab',0x490,0x4b0),('right_hstab',0x480,0x4e0)]}
        # Set the exact float atan input so caller/evaluator compare its actual angle.
        z=f32(math.tan(-local_angles[2]/f32(57.2957763671875)))
        self.floats(FRAME-0x688,[1.,0.,z]);self.u.mem_write(FRAME-0x6a0,struct.pack('<2d',0.,0.))
        u.reg_write(UC_X86_REG_R15,FRAME-0x9b4)
        u.emu_start(0x106c61edc,0x106c622dd,count=10000)
        result['vstab']=[self.read3(FRAME-0x4a0)[0],self.read3(FRAME-0x4d0)[0]]
        vertical_angle=mul_atan(z)
        z=f32(math.tan(-local_angles[3]/f32(57.2957763671875)))
        u.mem_write(BASE+0x1618,struct.pack('<3d',1.,0.,z))
        u.reg_write(UC_X86_REG_R15,FRAME-0x470)
        u.emu_start(0x106c622fb,0x106c62643,count=10000)
        result['fuselage']=[self.read_xmm(0)[0],self.read_xmm(9)[0]]
        return result,vertical_angle,mul_atan(z)


def mul_atan(z):
    return f32(f32(math.atan2(z,1.)) * -f32(57.2957763671875))


def verify_secondary():
    from tail_model import secondary_coefficients
    from polar_model import make_polar,round_polar
    from component_stages import runtime_secondary_areas
    m=SecondaryMachine();rng=random.Random(16639);failures=[];count=0
    def r(a,b):return f32(rng.uniform(a,b))
    for aircraft in ['f_16a_block_15_adf','saab_jas39c']:
        fm=json.loads(Path('references/fm-2.59.0.13/'+aircraft+'.blkx').read_text());ad=fm['Aerodynamics'];areas=runtime_secondary_areas(fm)
        for i in range(400):
            polars={}
            for name,key in [('hstab','HorStabPlane'),('vstab','VerStabPlane'),('fuselage','FuselagePlane')]:
                props=ad[key];polars[name]=round_polar(make_polar(props['Polar'],props['Span'],sum(props['Areas'].values()),r(0,2)))
            controls={n:[r(-25,25),r(-15,15),r(-.2,.2),r(-.1,.2),0.] for n in ['left_hstab','right_hstab','vstab']}
            angles=[r(-70,70) for _ in range(4)];body=[r(-70,70),r(-70,70)];prev=[r(-70,70),r(-70,70)]
            incidence=dict(hstab=r(-3,3),vstab=r(-3,3));inertia={k:r(-.2,.2) for k in ['hstab','vstab','fuselage']};dt=r(.01,.05)
            health={k:1. if i%2 else r(0,1) for k in areas}
            args=[polars,controls,angles,body,prev,incidence,inertia,dt,areas,health,i%2==0,r(-2,2),(r(.5,2),r(.5,2)),r(.5,2),r(0,.3)]
            actual,angles[2],angles[3]=m.prepared(*args)
            expected=secondary_coefficients(*args,coefficient_evaluator=m.coeff_eval)
            for name,value in actual.items():
                count+=1
                if value!=expected['coefficients'][name]:failures.append(dict(aircraft=aircraft,case=i,component=name,actual=value,expected=expected['coefficients'][name],args=args))
    report=dict(coefficient_checks=count,failures=failures,limitations='Horizontal and vertical contiguous producer spans, prepared mixer outputs and polar structs; original polar functions execute within the caller and as the coefficient oracle. Float32 output equality.')
    Path('analysis/tail-coefficient-validation.json').write_text(json.dumps(report,indent=2));print('SECONDARY',count,'FAILURES',len(failures));print(json.dumps(failures[:1],indent=2))
    if failures:raise SystemExit(1)


class IntegratedSecondaryMachine(SecondaryMachine):
    def __init__(self):
        super().__init__()
        from macho_scan import MachO
        m=MachO()
        for va,size in [(0x10198e000,0x1000),(0x1019e5000,0x2000)]:
            self.u.mem_map(va,size);self.u.mem_write(va,m.read(va,size))
        for address in [0x10198eab0,0x1019e52d0,0x1019e60c0]:
            self.u.hook_add(UC_HOOK_CODE,self.preparation_hook,begin=address,end=address)

    def preparation_hook(self,u,address,size,data):
        from polar_model import pack_polar
        if address==0x10198eab0:
            u.mem_write(u.reg_read(UC_X86_REG_RSI),pack_polar(self.prepared_properties['polars']['fuselage']))
        elif address==0x1019e60c0:
            name=['left_hstab','right_hstab','vstab'][self.control_index];self.control_index+=1
            self.floats(u.reg_read(UC_X86_REG_RDX),self.prepared_controls[name])
        else:self.xmm(0,[0.,0.]) # unused endpoint summary; actual mixer outputs supplied
        sp=u.reg_read(UC_X86_REG_RSP);ret=struct.unpack('<Q',u.mem_read(sp,8))[0]
        u.reg_write(UC_X86_REG_RSP,sp+8);u.reg_write(UC_X86_REG_RIP,ret)

    def call_secondary(self,p,s,wing,controls):
        from polar_model import pack_polar
        from component_stages import local_flow
        from component_assembly import add,mul
        self.reset();u=self.u;u.reg_write(UC_X86_REG_RSP,FRAME-0x3000)
        self.prepared_properties=p;self.prepared_controls=controls;self.control_index=0
        self.floats(0x107d6fc54,[p.get('travel_cap',1.)])
        u.mem_write(BASE+0x55a0,struct.pack('<Q',BASE+0x4000))
        u.mem_write(BASE+0x15e0,struct.pack('<3d',*s['omega']));u.mem_write(BASE+0x1618,struct.pack('<3d',*s['velocity']))
        self.floats(BASE+0x5320,s['cog']);self.floats(BASE+0x8454,s['body_angles']);self.floats(BASE+0x1680,s['previous_angles']);self.floats(BASE+0x1688,s['previous_wing_cl'])
        u.mem_write(BASE+0x7c0a,bytes([p.get('convert_aoa',False)]));u.mem_write(BASE+0x81ac,struct.pack('<I',2))
        for off,value in [(0x8138,p['span']),(0x8254,p['area']),(0x814c,p['sweep']),(0x8150,p['taper']),(0x8154,p['dihedral']),(0x81b0,p['downwash_coefficient']),(0x6f1c,p['incidence']['hstab']),(0x7124,p['incidence']['vstab']),(0x6f34,p['flow_inertia']['hstab']),(0x713c,p['flow_inertia']['vstab']),(0x7344,p['flow_inertia']['fuselage'])]:self.floats(BASE+off,[value])
        for key,off in dict(fuselage=0x83fc,left_main=0x8420,left_elevator=0x8424,right_main=0x8428,right_elevator=0x842c,v_main=0x8430,rudder=0x8434).items():self.floats(BASE+off,[p['areas'][key]])
        for key,off in dict(fuselage=0x18a0,left_main=0x18c4,left_elevator=0x18c8,right_main=0x18cc,right_elevator=0x18d0,v_main=0x18d4,rudder=0x18d8).items():self.floats(BASE+off,[s['health'][key]])
        self.floats(BASE+0x6f20,p['arms']['hstab']);self.floats(BASE+0x7330,p['arms']['fuselage'])
        vp=list(p['arms']['vstab']);vp[0]=add(vp[0],p['polars']['vstab']['aerCenterOffset']);vf=local_flow(vp,s['cog'],s['velocity'],s['omega'],s['disturbance'])
        for off,v in [(0x608,vp),(0x688,vf),(0x650,wing['points'][0]),(0x640,wing['points'][1]),(0x490,s['disturbance'][:2]),(0x480,[s['disturbance'][2]])]:self.floats(FRAME-off,v)
        for off,value in [(0x660,vf[0]),(0x590,vf[1]),(0x800,vf[2]),(0x790,mul(s['body_angles'][1],f32(.01745329238474369))),(0x8e0,mul(s['body_angles'][0],f32(.01745329238474369))),(0x5c0,s['dt']),(0x4b0,wing['cl'][0]),(0x4a0,wing['cl'][1]),(0x4c0,mul(s['density'],.5)),(0x998,p.get('horizontal_bias',0.)),(0x828,p.get('history_scales',(1.,1.))[0]),(0x6c0,p.get('history_scales',(1.,1.))[1]),(0x850,p.get('vertical_control_scale',1.)),(0x5f8,p.get('vertical_area_add',0.)),(0x9d0,p.get('vertical_area_scale',1.))]:self.floats(FRAME-off,[value])
        u.mem_write(FRAME-0x6f0,struct.pack('<d',p.get('vertical_lift_scale',1.)))
        for name,off in [('hstab',0xc58),('vstab',0x3f0)]:u.mem_write(FRAME-off,pack_polar(p['polars'][name]))
        u.reg_write(UC_X86_REG_R15,FRAME-0xc58)
        u.emu_start(0x106c60c95,0x106c62914,count=30000)
        return dict(forces={n:self.read3(FRAME-off) for n,off in [('left_hstab',0x7d0),('right_hstab',0x7c0),('vstab',0x7b0),('fuselage',0x718)]},points={n:self.read3(FRAME-off) for n,off in [('left_hstab',0x768),('right_hstab',0x758),('vstab',0x930),('fuselage',0x920)]},flows={n:self.read3(FRAME-off) for n,off in [('left_hstab',0x778),('right_hstab',0x630),('vstab',0x688),('fuselage',0xa00)]})


def verify_integrated():
    from tail_model import secondary_model
    from component_stages import runtime_secondary_areas
    from polar_model import make_polar,round_polar
    m=IntegratedSecondaryMachine();rng=random.Random(51639);fails=[];count=0
    def r(a,b):return f32(rng.uniform(a,b))
    for aircraft in ['f_16a_block_15_adf','saab_jas39c']:
        fm=json.loads(Path('references/fm-2.59.0.13/'+aircraft+'.blkx').read_text());ad=fm['Aerodynamics'];areas=runtime_secondary_areas(fm)
        for i in range(200):
            polars={}
            for name,key in [('hstab','HorStabPlane'),('vstab','VerStabPlane'),('fuselage','FuselagePlane')]:
                v=ad[key];polars[name]=round_polar(make_polar(v['Polar'],v['Span'],sum(v['Areas'].values()),r(0,2)))
            p=dict(polars=polars,span=r(8,16),area=r(20,60),sweep=r(10,70),taper=r(1,8),dihedral=r(-15,15),downwash_coefficient=[0.,f32(.8)][i%2],arms={n:[r(-5,5),r(-1,1),r(0,2)] for n in polars},incidence=dict(hstab=r(-2,2),vstab=r(-2,2)),flow_inertia={n:r(-.2,.2) for n in polars},areas=areas,convert_aoa=i%2==0,history_scales=(r(.5,2),r(.5,2)),vertical_control_scale=r(.5,2),vertical_area_add=r(0,.3),vertical_area_scale=r(.3,1),vertical_lift_scale=r(.5,2),horizontal_bias=r(-2,2))
            s=dict(velocity=[rng.uniform(30,600),rng.uniform(-150,150),rng.uniform(-50,50)],omega=[rng.uniform(-2,2) for _ in range(3)],cog=[r(-1,1) for _ in range(3)],density=r(.2,1.3),dt=r(.01,.05),body_angles=[r(-50,50),r(-30,30)],previous_angles=[r(-50,50),r(-30,30)],previous_wing_cl=[r(-2,2),r(-2,2)],disturbance=[r(-5,5) for _ in range(3)],health={k:1. if i%2 else r(0,1) for k in areas})
            wing=dict(points=[[r(-1,1),r(-.3,.3),r(1,4)],[r(-1,1),r(-.3,.3),r(-4,-1)]],cl=[r(-2,2),r(-2,2)])
            controls={n:[r(-25,25),r(-15,15),r(-.2,.2),r(-.1,.2),0.] for n in ['left_hstab','right_hstab','vstab']}
            actual=m.call_secondary(p,s,wing,controls);expected=secondary_model(p,s,wing,controls);count+=1
            for kind in ['flows','points','forces']:
                for name in actual[kind]:
                    if actual[kind][name]!=expected[kind][name]:fails.append(dict(aircraft=aircraft,case=i,kind=kind,component=name,actual=actual[kind][name],expected=expected[kind][name],inputs=(p,s,wing,controls)))
    report=dict(contiguous_secondary_executions=count,failures=fails,limitations='0x106c60c95..62914 executed unchanged except prepared polar and mixer providers. Original coefficient helpers run inside the span. Vertical flow prepared by independently verified slice. Jet/no-propeller path, finite inputs. Host libm hooks.')
    Path('analysis/tail-integrated-validation.json').write_text(json.dumps(report,indent=2));print('INTEGRATED',count,'FAILURES',len(fails));print(json.dumps(fails[:1],indent=2))
    if fails:raise SystemExit(1)


if __name__=='__main__':
    main()
    verify_secondary()
    verify_integrated()
