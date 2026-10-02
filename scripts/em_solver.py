import json
import math
import time
from types import SimpleNamespace
from collections import deque
from functools import lru_cache
from pathlib import Path

import numpy as np
from em_roots import checked_root
from scipy.optimize import least_squares, brentq, lsq_linear
from em_cancellation import check as check_cancel
from em_pitch_response import response as pitch_response, recover_attempt as recover_pitch_response, KINDS as PHYSICAL_KINDS

from em_backend import activate
BACKEND = activate()

from aircraft_model import prepare, evaluate, condition_properties,at_sweep,replay_propulsion
from air_state import cache, world_to_body_air
from body_dynamics import preprocess_angular_rate, angular_acceleration, G
from component_assembly import f32, mul, add
from control_mixer import density_at_height
from engine_supply import selected_properties as engine_properties, fuel_properties, healthy_running_owner_step,available_fuel,mechanical_multiplier
from engine_commands import afterburner_command
from jet_model import prepare as prepare_jet,prepare_nozzle, mode, table, selected_nozzle,scalar_update
from jet_nozzle import prepare as prepare_general_nozzle,evaluate as evaluate_nozzles
from aircraft_catalog import LazyCatalog,catalog,load as load_aircraft,is_prop,REFERENCE
from jet_catalog import engines as installed_engines,fuel_capacities
from prop_catalog import mass_state as prop_mass_state
from prop_em import PropellerEnsemble
from wing_sweep import schedule as sweep_schedule,available as available_sweep
from instructor_envelope import profile as instructor_profile
from kinematics import airborne_step, altitude_velocity_correction
from mass_model import aircraft_properties, evaluate as mass_evaluate, tank_configuration
from polar_runtime import flap_polar, evaluate as mach_polar
from primary_controls import selected_properties as control_properties, authority_ranges, steady_commands

ROOT = Path(__file__).resolve().parents[1]
AIRCRAFT = LazyCatalog()
DEFAULTS = dict(aircraft=REFERENCE, altitude_m=0., fuel_percent=30., throttle=1.1,
                afterburner=True, torque_gyro=False, engine_control_mode='quasi_steady', aircraft_trim_mode='discrete', trim_mode='optimized', trim_limit=1., fixed_trim=[0., 0., 0.],
                extra_mass_kg=0., default_ammunition=False, ammunition_vehicle='', speed_min_kmh=100., speed_max_kmh=1300., max_load_g=None,
                speed_samples=9, load_samples=9, structural_limits=True, timestep_hz=48.,low_speed_load_cap=False,global_load_cap_g=64.,reference_load_cap=False,aircraft_search_region=True,
                sampling='adaptive',sep_tolerance_mps=.5,surface_resolution=601,sep_contour_levels_mps=[100.,0.,-100.,-200.,-400.],sweep_percent=0.,flaps_percent=0.,instructor=True,
                aircraft_settings={},compare_instructor=False,entries=None,instructor_model='steady',instructor_authority_mode='direct',trim_solver_mode='nested',roll_leveling=False,turn_response_mode='settled',mach_curve_mode='native')

# Backend-only opt-in: mach_curve_mode='continuous' enables the selective tanh
# approximation; 'native' (default) keeps native arithmetic and gap handling.
# Preliminary aircraft force estimates define the default chart search region.
# The cap is predetermined from aircraft data; 64 g is an outer guard only.
# The former frozen reference envelope remains an opt-in for reproduction.


AIRCRAFT_SETTINGS = frozenset(('altitude_m','fuel_percent','throttle','afterburner',
    'trim_mode','trim_limit','fixed_trim','extra_mass_kg','default_ammunition','ammunition_vehicle','structural_limits','turn_response_mode',
    'timestep_hz','sweep_percent','flaps_percent','instructor','instructor_model','instructor_authority_mode','engine_control_mode','aircraft_trim_mode','trim_solver_mode','torque_gyro','roll_leveling','mach_curve_mode'))


def contour_levels(values):
    if (not isinstance(values,list) or len(values)>16 or any(isinstance(v,bool) or not isinstance(v,(int,float))
            or not math.isfinite(v) or abs(v)>2000 for v in values) or len(set(values))!=len(values)):
        raise ValueError('SEP contours must be up to 16 unique values between -2000 and 2000 m/s')
    return [float(v) for v in values]


def aircraft_settings(config, name):
    if name not in config['aircraft']:raise ValueError('Aircraft is not selected: '+name)
    return settings(dict(config, **dict(config.get('aircraft_settings',{}).get(name,{}),
                                        aircraft=[name],aircraft_settings={})))


def settings(values=None):
    if values and values.get('entries') is not None:
        from em_entries import entry_settings
        return entry_settings(values)
    result=dict(DEFAULTS); result.update(values or {})
    unknown=set(result)-set(DEFAULTS)
    if unknown: raise ValueError('Unknown setting: '+', '.join(sorted(unknown)))
    if not isinstance(result['aircraft'], list) or not result['aircraft'] or len(result['aircraft'])>2:
        raise ValueError('Select one or two aircraft')
    if any(a not in AIRCRAFT for a in result['aircraft']) or len(set(result['aircraft']))!=len(result['aircraft']):
        raise ValueError('Unknown or repeated aircraft')
    if (not values or 'sep_contour_levels_mps' not in values) and all(is_prop(a) for a in result['aircraft']):
        result['sep_contour_levels_mps']=[10.,0.,-10.,-20.,-40.]
    for name in result['aircraft']:
        if not AIRCRAFT[name]['supported']:raise ValueError(AIRCRAFT[name]['name']+': '+AIRCRAFT[name]['reason'])
    limits={'altitude_m': (0,18000), 'fuel_percent': (1,100), 'throttle': (0,1.1),
            'trim_limit': (0,1), 'extra_mass_kg': (0,5000), 'speed_min_kmh': (100,2500),
            'speed_max_kmh': (200,2600), 'max_load_g': (1.1,20), 'global_load_cap_g': (1.1,64),
            'speed_samples': (7,129), 'load_samples': (5,49), 'timestep_hz': (30,120),
            'sep_tolerance_mps':(.05,5.),'surface_resolution':(201,1201),'sweep_percent':(0,100),'flaps_percent':(0,100)}
    for key,(low,high) in limits.items():
        value=result[key]
        if key=='max_load_g' and value is None:continue
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not low<=value<=high:
            raise ValueError(f'{key} must be between {low} and {high}')
        result[key]=float(value)
    result['flaps_percent']=int(math.floor(result['flaps_percent']+.5))
    for key in ['speed_samples','load_samples','surface_resolution']:
        if int(result[key])!=result[key]: raise ValueError(key+' must be an integer')
        result[key]=int(result[key])
    result['sep_contour_levels_mps']=contour_levels(result['sep_contour_levels_mps'])
    if result['speed_min_kmh']>=result['speed_max_kmh']: raise ValueError('Maximum speed must exceed minimum speed')
    for key in ['default_ammunition','afterburner','structural_limits','instructor','compare_instructor','torque_gyro','roll_leveling','low_speed_load_cap','reference_load_cap','aircraft_search_region']:
        if not isinstance(result[key],bool): raise ValueError(key+' must be true or false')
    if not isinstance(result['ammunition_vehicle'],str):raise ValueError('ammunition_vehicle must be a vehicle ID')
    # The frozen chart domain is always intersected with this aircraft's
    # native speed and wing-force limits, including per-aircraft overrides.
    if result['reference_load_cap']:result['structural_limits']=True
    if result['compare_instructor'] and len(result['aircraft'])!=1:
        raise ValueError('Select one aircraft to compare Instructor on/off')
    if result['engine_control_mode'] not in ('automatic','quasi_steady'):raise ValueError('Unknown engine control mode')
    if result['mach_curve_mode'] not in ('native','continuous'):raise ValueError('Unknown Mach curve mode')
    
    if result['aircraft_trim_mode']!='discrete':raise ValueError('Aircraft trim mode is unavailable')
    if result['instructor_model']!='steady':raise ValueError('Only the static Instructor boundary is supported')
    if result['instructor_authority_mode']!='direct':raise ValueError('Instructor authority mode is unavailable')
    if result['trim_solver_mode']!='nested':raise ValueError('Trim solver mode is unavailable')
    if result['trim_mode'] not in ('optimized','fixed'): raise ValueError('Unknown trim mode')
    if result['sampling'] not in ('adaptive','regular'):raise ValueError('Unknown sampling mode')
    if result['turn_response_mode']!='settled':raise ValueError('Turn response mode is unavailable')
    if result['sampling']=='regular' and result['max_load_g'] is None:raise ValueError('Regular diagnostic sampling requires a load range; use adaptive for the automatic envelope')
    if not isinstance(result['fixed_trim'],list) or len(result['fixed_trim'])!=3:
        raise ValueError('Fixed trim must contain roll, pitch and yaw')
    if any(isinstance(t,bool) or not isinstance(t,(int,float)) or not math.isfinite(t) or abs(t)>1 for t in result['fixed_trim']):
        raise ValueError('Each fixed trim must be between -1 and 1')
    result['fixed_trim']=[float(t) for t in result['fixed_trim']]
    overrides=result['aircraft_settings']
    if not isinstance(overrides,dict) or set(overrides)-set(result['aircraft']):
        raise ValueError('Aircraft settings must map selected aircraft to their conditions')
    normalized={}
    for name,conditions in overrides.items():
        if not isinstance(conditions,dict) or set(conditions)-AIRCRAFT_SETTINGS:
            raise ValueError('Unknown aircraft condition for '+name)
        resolved=settings(dict(result,**dict(conditions,aircraft=[name],aircraft_settings={})))
        normalized[name]={key:resolved[key] for key in sorted(AIRCRAFT_SETTINGS)}
    result['aircraft_settings']=normalized
    if len(result['aircraft'])==1 and not AIRCRAFT[result['aircraft'][0]].get('has_flaps',False):
        result['flaps_percent']=0.
    for name in result['aircraft']:
        condition=dict(result,**normalized.get(name,{}))
        # Mixed batches carry shared defaults plus per-aircraft engine modes.
        if is_prop(name) and condition['engine_control_mode']!='quasi_steady':
            raise ValueError('Propeller engine mode is unavailable')
        if condition['default_ammunition']:
            from aircraft_ammunition import selected_mass
            selected_mass(name,condition['ammunition_vehicle'])
    return result


def turn_geometry(alpha_deg, bank_deg, speed, load, dt, ias_u, sideslip_deg=0.):
    alpha,bank=map(math.radians,(alpha_deg,bank_deg))
    sa,ca,sb,cb=math.sin(alpha),math.cos(alpha),math.sin(bank),math.cos(bank)
    forward=np.array([ca,-sa,0.]); normal=np.array([sa,ca,0.])
    side=np.array([0.,0.,1.])
    if sideslip_deg:
        beta=math.radians(sideslip_deg);ss,cs=math.sin(beta),math.cos(beta)
        forward=np.array([ca*cs,-sa*cs,ss]);side=np.array([-ca*ss,sa*ss,cs])
    up=cb*normal-sb*side
    lateral=sb*normal+cb*side
    turn_rate=float(G)*math.sqrt(max(0.,load*load-1.))/speed

    x,y,z=-up*math.sin(turn_rate*dt*.5); w=math.cos(turn_rate*dt*.5)
    r00=1-2*(y*y+z*z); r10=2*(x*y+w*z); r20=2*(x*z-w*y)
    r11=1-2*(x*x+z*z); r12=2*(y*z-w*x)
    yaw=math.atan2(-r20,r00); pitch=math.asin(max(-1.,min(1.,r10))); roll=math.atan2(-r12,r11)
    midpoint=-np.array([roll,yaw,pitch])/dt
    damping=max(1.-float(f32(ias_u))*1e-5,.8)
    stored=midpoint*(2./(1.+damping))

    quaternion=[f32(math.sin(bank*.5)*math.cos(alpha*.5)),
                f32(-math.sin(bank*.5)*math.sin(alpha*.5)),
                f32(math.cos(bank*.5)*math.sin(alpha*.5)),
                f32(math.cos(bank*.5)*math.cos(alpha*.5))]
    if sideslip_deg:


        sx,cx=math.sin(bank*.5),math.cos(bank*.5)
        sy,cy=math.sin(beta*.5),math.cos(beta*.5)
        sz,cz=math.sin(alpha*.5),math.cos(alpha*.5)
        quaternion=list(map(f32,[sx*cy*cz+cx*sy*sz,cx*sy*cz-sx*cy*sz,
                                cx*cy*sz+sx*sy*cz,cx*cy*cz-sx*sy*sz]))
    return dict(forward=forward, normal=normal, side=side, up=up, lateral=lateral,
                omega=stored, quaternion=quaternion, turn_rate=turn_rate)


class EngineUnit:
    def __init__(self, model, mass, config, instance=None):
        self.model,self.mass,self.config=model,mass,config
        fm=model['fm'];instance=instance or installed_engines(fm)[0][1]
        boost=instance['Afterburner']
        self.boost_type=boost['Type'];self.boost_controllable=boost.get('IsControllable',True)
        self.afterburner=afterburner_command(config['afterburner'],self.boost_type,self.boost_controllable)
        self.jet=prepare_jet(instance['Main']); self.dt=f32(1/config['timestep_hz'])
        self.ep=engine_properties(instance);system=instance['Main'].get('FuelSystemNum',0)
        self.fp=fuel_properties(fm['Mass'],system)
        self.system_fuel=mass.get('fuel_by_system',[mass['fuel_mass']])[system]
        import re
        self.nozzles=[prepare_general_nozzle(instance[k]) for k in sorted((k for k in instance if re.fullmatch(r'Nozzle\d+',k)),key=lambda k:int(k[6:]))]


        self.warm_nozzle=dict(position=[0.,0.,0.],ratio=0.,maximum=2147440000.,flaps=[0.,1.,1.,1.],
                         airbrake=[(0.,0.,[0.,0.])],vtol=[(0.,0.,[0.,0.])],reverse=[(0.,0.,[0.,0.])])
        try:self.nozzle=prepare_nozzle(instance['Nozzle0'])
        except (KeyError,ValueError):self.nozzle=None
        self.rho=density_at_height(f32(config['altitude_m']))
        self.accumulator=min(self.fp['capacity'],self.system_fuel)
        self.states,self.summary=self.settle(250.)
        self.local_cycles={self.nominal_target(250.):self.states}
        self.factors=[mode(self.jet,f32(s['omega']/self.jet['max_omega']))[0] for s in self.states]

    def settle(self,body_u):
        config=self.config
        state=dict(omega=mul(.6,self.jet['max_omega']), health=1., cylinders=self.ep['cylinders'],
                   mechanical=1., extra_amplitude=0., torque=0., friction=0., throttle=f32(config['throttle']),
                   running=7, afterburner=self.afterburner, vtol=0., reverse=0., rpm_limit_scale=1.,
                   elapsed=100., inactive_elapsed=0., stop_reason=0)
        seed=183193; recent=deque(maxlen=32); samples=[]; period=0
        burn=round(40/self.dt); window=round(20/self.dt)
        for tick in range(burn+window):
            old=dict(state,_seed=seed)
            r=healthy_running_owner_step(self.ep,self.jet,self.warm_nozzle,self.fp,state,[body_u,0.,0.],
                config['altitude_m'],self.mass['cog'],self.dt,seed,self.system_fuel,self.accumulator)
            state,seed=r['state'],r['seed']
            old['_next_omega']=state['omega']
            record=(state['omega'],state['mechanical'],seed,old)
            recent.append(record)
            if len(recent)==32:
                rows=list(recent)
                for p in [1,2,3,4,8]:
                    if all(rows[-j][:3]==rows[-j-p][:3] for j in range(1,25)):
                        period=p; samples=[r[3] for r in rows[-p:]]; break
                if period: break
            if tick>=burn:samples.append(old)
        summary=dict(policy='fixed point' if period==1 else f'{period}-step cycle' if period else '20 s seeded mean after 40 s warm-up',
                          period_steps=period or None,samples=len(samples),seed=183193,
                          rpm_mean=float(np.mean([s['omega']*60/(2*math.pi) for s in samples])),
                          rpm_range=[float(min(s['omega'] for s in samples)*60/(2*math.pi)),float(max(s['omega'] for s in samples)*60/(2*math.pi))])
        return samples,summary

    def nominal_target(self,body_u):
        state=self.states[0]
        r=scalar_update(self.jet,self.rho,body_u,state['omega'],
            min(f32(state.get('effective_throttle',self.config['throttle'])/self.jet['throttle_scale']),1.),self.dt,
            afterburner=self.afterburner,fuel_available=available_fuel(self.fp,self.system_fuel,self.accumulator,1.,self.dt))
        return r['target_omega'],r['active']

    def phase_states(self,body_u):


        key=self.nominal_target(body_u)
        if key not in self.local_cycles:
            if len(self.local_cycles)>512:self.local_cycles.clear()
            self.local_cycles[key]=self.settle(body_u)[0]
        return self.local_cycles[key]

    def scalar_phase(self,body_u,state):
        health,_,_=mechanical_multiplier(self.ep,state['omega'],state['health'],state['cylinders'],
            state['mechanical'],0.,state['torque'],state['friction'],self.dt,state['_seed'])
        supply=available_fuel(self.fp,self.system_fuel,self.accumulator,1.,self.dt)
        throttle=state.get('effective_throttle',self.config['throttle'])
        scalar=scalar_update(self.jet,self.rho,body_u,state['omega'],min(f32(throttle/self.jet['throttle_scale']),1.),self.dt,
            afterburner=self.afterburner,fuel_available=supply,health=health,
            shaft_fraction=mul(f32(state['omega']),self.ep['inverse_omega']))
        next_omega=min(max(0.,scalar['next_omega']),mul(self.ep['omega_limit'],f32(state['rpm_limit_scale'])))
        if next_omega!=state['_next_omega']:
            raise ValueError('Speed-dependent fuel-flow-limited RPM requires local engine settling')
        return scalar

    @lru_cache(maxsize=2048)
    def vectors(self, body_u, flaps=0.):
        ias=mul(body_u,f32(math.sqrt(f32(self.rho/f32(1.225)))))
        results=[]
        for state in self.phase_states(body_u):
            scalar=self.scalar_phase(body_u,state)
            results.append(evaluate_nozzles(self.nozzles,scalar['thrust'],self.mass['cog'],ias,flaps=flaps))
        force=np.mean([r['force'] for r in results],axis=0).tolist()
        moment=np.mean([r['moment'] for r in results],axis=0).tolist()
        return force,moment


class EngineEnsemble:
    def __init__(self,model,mass,config):
        installed=installed_engines(model['fm'])
        rocket_only=bool(installed) and all(e['Main']['Type']=='Rocket' and not e.get('Booster',False) for _,e in installed)
        if rocket_only:
            from rocket_em import RocketUnit
            self.units=[RocketUnit(model,mass,config,e) for _,e in installed]
        else:self.units=[EngineUnit(model,mass,config,e) for _,e in installed if e['Main']['Type']=='Jet']
        if not self.units:raise ValueError('No installed supported direct-thrust engines')
        self.summary=dict(self.units[0].summary)
        self.summary.update(engine_count=len(self.units),engines=[u.summary for u in self.units],
                            nozzle_policy='Normal forward flight; VTOL, reverse and thrust-vectoring commands zero')
        if len(self.units)>1:self.summary['policy']=str(len(self.units))+' engines · '+', '.join(dict.fromkeys(u.summary['policy'] for u in self.units))

    def __getattr__(self,name):
        units=self.__dict__.get('units')
        if units is None:raise AttributeError(name)
        return getattr(units[0],name)

    @lru_cache(maxsize=2048)
    def vectors(self,body_u,flaps=0.):
        force=[0.]*3;moment=[0.]*3
        for unit in self.units:
            f,m=unit.vectors(body_u,flaps)
            for i in range(3):force[i]+=f[i];moment[i]+=m[i]
        return force,moment


def command_allocation(properties, coordinate, ranges, config):
    available=properties['trim_available']
    if config['trim_mode']=='optimized':
        low_trim=[-config['trim_limit'] if a else 0. for a in available]
        high_trim=[config['trim_limit'] if a else 0. for a in available]
    else: low_trim=high_trim=[t if a else 0. for t,a in zip(config['fixed_trim'],available)]
    ends=list(zip(steady_commands(properties,[-1.]*3,low_trim,ranges),steady_commands(properties,[1.]*3,high_trim,ranges)))
    bounds=[sorted((a,b)) for a,b in ends]
    desired=list(map(f32,coordinate))
    trim=[]; sticks=[]
    for i,(d,(lo,hi)) in enumerate(zip(desired,ranges)):
        raw=-d if i==1 and properties['invert_elevator'] else d
        t=max(-config['trim_limit'],min(config['trim_limit'],raw)) if config['trim_mode']=='optimized' and available[i] else low_trim[i]
        delta=raw-t
        denominator=1.-abs(t)*(1. if delta*t>0 else -1. if delta*t<0 else 0.)
        v=delta/denominator if denominator else 0.
        stick=2.*(v-lo)/(hi-lo)-1. if hi>lo else 0.
        sticks.append(f32(max(-1.,min(1.,stick)))); trim.append(f32(t))
    delivered=steady_commands(properties,sticks,trim,ranges)
    return dict(commands=desired,allocated_commands=delivered,sticks=sticks,trim=trim,ranges=ranges,
                bounds=bounds,reachable=all(lo-2e-7<=d<=hi+2e-7 for d,(lo,hi) in zip(desired,bounds)),
                allocation_error=max(abs(a-b) for a,b in zip(delivered,desired)))


class TrimSolver:


    trim_bounds=((-89.,-15.,-1.,-1.,-1.),(89.,89.7,1.,1.,1.))

    def __init__(self, name, config):
        self.name=name; self.config=aircraft_settings(settings(config),name)


        self.fm=dict(load_aircraft(name),RollLeveling=self.config['roll_leveling'])
        self.flaps=f32(self.config['flaps_percent']/100.)
        from flap_model import profile as flap_profile
        self.flap_profile=flap_profile(self.fm)
        self.instructor_profile=instructor_profile(self.fm,self.config)
        self.model=at_sweep(prepare(self.fm),self.config['sweep_percent']/100.); self.controls=control_properties(self.fm)
        from em_mach_events import configure_model
        configure_model(self.model,self.config['mach_curve_mode'])
        self.model['aircraft_trim_mode']=self.config['aircraft_trim_mode']
        self.sweep_rows=sweep_schedule(self.fm);self.has_sweep=len(self.model['wing_family'])>1
        self.is_prop=is_prop(name)


        self.gear=1. if self.is_prop and not self.fm['AvailableControls'].get('hasGearControl',True) else 0.
        from aircraft_ammunition import selected_mass, selected_payloads, profile as ammunition_profile
        ammunition_mass=selected_mass(name,self.config['ammunition_vehicle']) if self.config['default_ammunition'] else 0.
        added_mass=self.config['extra_mass_kg']
        weapon_payloads=selected_payloads(name,self.config['ammunition_vehicle']) if self.config['default_ammunition'] else []
        if self.is_prop:self.mass=prop_mass_state(name,self.config['fuel_percent'],added_mass,payloads=weapon_payloads)
        else:
            fuel=[f32(capacity*self.config['fuel_percent']/100.) for capacity in fuel_capacities(self.fm)]
            payloads=[*weapon_payloads,*([dict(mass=added_mass,position=self.fm['Mass']['CenterOfGravity'])] if added_mass else [])]
            self.mass=mass_evaluate(aircraft_properties(self.fm),fuel,payloads=payloads)
            self.mass['fuel_by_system']=fuel
        self.mass['ammunition_mass_kg']=ammunition_mass
        self.mass['additional_mass_kg']=self.config['extra_mass_kg']
        if self.config['default_ammunition']:
            self.mass['ammunition']=ammunition_profile(name,self.config['ammunition_vehicle'])
            self.mass['mass_policy']='Internal fuel and default weapon payloads at their attachments; manual additional mass at configured CG'
        self.weight=self.mass['mass']*float(G); self.dt=f32(1/self.config['timestep_hz'])
        self.engine=PropellerEnsemble(name,self.model,self.mass,self.config) if self.is_prop else EngineEnsemble(self.model,self.mass,self.config)
        self.instructor_boundaries={}
        self.instructor_trim_entries={}
        self.sideslip_attitude_deg=0.
        self._sideslip_solvers={}

    @property
    def numeric_parameters(self):
        from em_numeric_core import TrimParameters
        values=(self.mass['mass'],self.weight,tuple(self.mass['inertia']),self.dt)
        previous=self.__dict__.get('_numeric_parameters')
        if previous is None or self.__dict__['_numeric_parameter_values']!=values:
            previous=TrimParameters(*values)
            self._numeric_parameters=previous;self._numeric_parameter_values=values
        return previous

    def at_sideslip(self,angle):
        angle=float(angle)
        if angle==self.sideslip_attitude_deg:return self
        if angle not in self._sideslip_solvers:
            import copy
            view=copy.copy(self);view.sideslip_attitude_deg=angle
            view.instructor_boundaries={};view._sideslip_solvers={}
            view.instructor_trim_entries={}
            view.__dict__.pop('_physical_sideslip_recovery',None)
            view.__dict__.pop('_minimum_instructor_speed',None)
            self._sideslip_solvers[angle]=view
        return self._sideslip_solvers[angle]

    def point_value(self,point):
        view=self.at_sideslip(point.get('sideslip_attitude_deg',0.))
        if self.is_prop and point.get('propulsion'):
            view=view.with_prop_controls(point['propulsion']['controls'])
        return view.operating_point(point['speed_kmh']/3.6,point['load_g'],point['solution'],canonical_propulsion=self.is_prop)

    def with_prop_controls(self,controls):
        import copy
        view=copy.copy(self);view.engine=self.engine.with_controls(controls)
        view.__dict__.pop('_trim_predictor',None)
        view.__dict__.pop('_interior_curve',None)
        view.__dict__.pop('_physical_sideslip_recovery',None)
        view.instructor_boundaries={};view._sideslip_solvers={}


        same_controls=controls==(self.engine.fixed_controls or
            ((self.engine.automatic_controls)))
        view.instructor_trim_entries=self.instructor_trim_entries if same_controls else {}
        view.__dict__.pop('_minimum_instructor_speed',None)
        return view

    def instructor_at(self,value,speed,load):
        from instructor_envelope import controller_limits
        return controller_limits(self,value,speed)

    def derivative_branch(self,value):
        if '_derivative_branch' in value:return value['_derivative_branch']
        r=value['result'];mach=r['air']['mach']
        _,wing,secondary=condition_properties(self.model,mach,value['flaps'])
        def polar_branch(p,a):
            if p['aoaLineL']<=a<=p['aoaLineH']:return ('linear',)
            sign=1. if add(a,.01)>=p['aoaLineH'] else -1.
            critical=p['aoaCritH' if sign>0 else 'aoaCritL']
            distance=mul(a-critical,sign);sa=mul(sign,a)
            if distance<=0:return ('precrit',sign)
            return ('postcrit',sign,distance<p['parabAngle'],sa<=p['maxDistAng'],sa<=max(40.,p['maxDistAng']),sa<=90.,sa<=140.)
        blend=r['wing']['blend']
        key=[mach,0 if blend==0 else 2 if blend==1 else 1]
        if self.fm.get('RollLeveling',True):key.append(-1 if r['air']['alpha']<=-12. else 1 if r['air']['alpha']>=12. else 0)
        key.extend(polar_branch(wing,c['aoa']) for c in r['wing']['coefficients'])
        for name,angle in r['tail']['effective_angles'].items():
            polar=secondary['polars']['hstab' if 'hstab' in name else name]
            key.append(polar_branch(polar,angle))
        value['_derivative_branch']=tuple(key)
        return value['_derivative_branch']

    def boundary(self,speed_kmh,low,high,continuation=True,scalar_fallback=True,
                 candidate_kinds=None,max_iterations=24,trial_cycle_seconds=None):
        angle=low.get('sideslip_attitude_deg',self.sideslip_attitude_deg)
        if angle!=self.sideslip_attitude_deg:
            view=self.at_sideslip(angle)
            other=view.solve(speed_kmh,high['load_g'],high['solution'],exhaustive=False)
            return view.boundary(speed_kmh,low,other,continuation=continuation,scalar_fallback=scalar_fallback,
                                 candidate_kinds=candidate_kinds,max_iterations=max_iterations,
                                 trial_cycle_seconds=trial_cycle_seconds)
        pass
        from em_constraint_bracket import refine as refine_physical_bracket
        bracketed=refine_physical_bracket(self,speed_kmh,low,high)
        if bracketed:return bracketed
        if candidate_kinds is None or 'Instructor pitch' in candidate_kinds:
            from em_level_bracket import bracket as level_controller_bracket
            bracketed=level_controller_bracket(self,speed_kmh,low,high)
            if bracketed:return bracketed
        if (low['valid'] and ('reversed pitch response' in high['reasons'] or
                high.get('continuation_limit_kind')=='pitch response' or
                candidate_kinds and 'pitch response' in candidate_kinds)):
            from em_pitch_limit import limit as normal_pitch_limit
            response_limit=normal_pitch_limit(self,speed_kmh,low)
            if response_limit:return response_limit
        near_level=(low['valid'] and low['load_g']<1.05 and
                    (high['load_g']<1.3 or low['stall_margin_deg']<2. or
                     low['authority_margin']<.1 or
                     self.config['instructor'] and low.get('instructor',{}).get('envelope_margin',1.)<.02))
        if near_level:


            from em_level_limit import near_level_boundary
            point=near_level_boundary(self,speed_kmh,low,high,candidate_kinds,max_iterations)
            if point:return point
        strength={'CritOverload':self.model['geometry']['strength']['force']}
        candidates=[]
        control_seed=high if high.get('converged') else low
        for i in range(2,5):
            lo,hi=high.get('control_bounds',[[-1.,1.]]*3)[i-2]


            command=control_seed['solution'][i]
            if i==3 or command>hi*.8 or command<lo*.8:candidates.append(('control',i,1. if command>=0 else -1.))
        candidates.append(('stall',0,0.))
        if self.config['instructor']:candidates.insert(0,('Instructor pitch',0,0.))
        if self.config['structural_limits'] and max(high['wing_load_ratios'])>.95:
            candidates.append(('wing force',0,0.))
        if ('reversed pitch response' in high['reasons'] or
                high.get('continuation_limit_kind')=='pitch response' or
                candidate_kinds and 'pitch response' in candidate_kinds):
            candidates.append(('pitch response',0,0.))


        crossed=PHYSICAL_KINDS
        crossed_kind=crossed.get(high['reasons'][0]) if high['converged'] and len(high['reasons'])==1 else None


        crossed_kind=crossed_kind or high.get('continuation_limit_kind')
        if crossed_kind:candidates.sort(key=lambda candidate:candidate[0]!=crossed_kind)
        if candidate_kinds is not None:
            candidates=[candidate for candidate in candidates if candidate[0] in candidate_kinds]
        def isolated_limit(point,kind):


            return (low['load_g']<=point['load_g']<=high['load_g']
                and (kind=='stall' or point['stall_margin_deg']>.004)
                and (kind=='control' or point['authority_margin']>6e-5)
                and (kind=='pitch response' or (point.get('pitch_response') or {}).get('margin',0.)>4e-6)
                and (kind=='wing force' or not self.config['structural_limits'] or max(point['wing_load_ratios'])<.99990)
                and (kind=='Instructor pitch' or not self.config['instructor'] or point['instructor']['envelope_margin']>.0001))
        if self.is_prop and self.engine.automatic and low['valid']:


            nearby=[]
            if low['stall_margin_deg']<1.:nearby.append('stall')
            if low['authority_margin']<.05:nearby.append('control')
            if self.config['structural_limits'] and max(low['wing_load_ratios'])>.97:nearby.append('wing force')
            if self.config['instructor'] and (low.get('instructor') or {}).get('envelope_margin',1.)<.02:
                nearby.append('Instructor pitch')
            nearby=[kind for kind in nearby if candidate_kinds is None or kind in candidate_kinds]
            nearby.sort(key=lambda kind:kind!=crossed_kind)
            if nearby:
                from em_continuation import TrimCurve
                point=TrimCurve(self,speed_kmh,low).limit(low,nearby[0])
                if point and isolated_limit(point,nearby[0]):return point
        from em_load_limits import ceiling
        lower=np.array([*self.trim_bounds[0],1.]);upper=np.array([*self.trim_bounds[1],ceiling(self,speed_kmh)])
        answers=[]
        self.boundary_diagnostics=[]


        compare_prop_controls=self.is_prop and not self.engine.automatic
        boundary_force_goal=1e-6 if compare_prop_controls else 2e-4
        boundary_rate_goal=2e-6 if compare_prop_controls else 5e-5
        pitch_inset=min(2e-6,max(0.,low['instructor']['envelope_margin']*.5)) if self.config['instructor'] and low.get('instructor') else 2e-6
        response_inset=min(2e-6,(low.get('pitch_response') or {}).get('margin',4e-6)*.5)
        attempts=[(kind,axis,target,seed) for seed in [high,low] for kind,axis,target in candidates]


        scalar_wing_bracket=(self.is_prop and self.engine.automatic and crossed_kind=='wing force'
            and low['valid'] and high['converged'] and set(high['reasons'])<= {'wing force limit'}
            and max(low['wing_load_ratios'])<=.99995<max(high['wing_load_ratios']))
        if scalar_wing_bracket:attempts=[]
        if self.config['instructor']:


            attempts=[(kind,axis,inset,seed) for kind,axis,target in candidates
                      for inset in ([target,5e-5] if kind=='Instructor pitch' else [target])
                      for seed in [high,low]]


        attempts.sort(key=lambda item:item[0]=='pitch response' and item[3] is high)
        for kind,axis,target,seed in attempts:
            if any(p['envelope_limit']['kind']==kind and
                   p['envelope_limit']['axis']==(axis if kind=='control' else None) for p in answers):continue
            memo={};equilibrium_solver=self
            constraint_tolerance=2e-5 if kind=='stall' else 5e-7 if kind=='Instructor pitch' and not target else 5e-6
            active_pitch_inset=target or pitch_inset
            def evaluate_at(z,frozen=None):
                key=tuple(z)
                if frozen is not None or key not in memo:
                    value=equilibrium_solver.operating_point(speed_kmh/3.6,z[5],z[:5],propulsion_override=frozen,
                        cycle_seconds=(trial_cycle_seconds if trial_cycle_seconds is not None else
                                       20. if self.is_prop and self.engine.automatic and self.engine.prefer_short_search else 60.))
                    if kind=='control':
                        bounds_at_point=self.instructor_at(value,speed_kmh/3.6,z[5])['control_bounds'] if self.config['instructor'] else value['allocation']['bounds']
                        bound=bounds_at_point[axis-2][1 if target>0 else 0]
                        margin=z[axis]-(bound-target*3e-5)
                    elif kind=='stall':margin=(value['stall_margin']-.002)/10.
                    elif kind=='Instructor pitch':margin=self.instructor_at(value,speed_kmh/3.6,z[5])['envelope_margin']-active_pitch_inset
                    elif kind=='pitch response':margin=pitch_response(equilibrium_solver,value)['margin']-response_inset
                    else:
                        if self.is_prop:margin=max(value['maximum_wing_load_ratios'])-.99995
                        else:
                            forces=value['result']['component_forces'];positive_limit=max(strength['CritOverload'])
                            margin=max(forces[side][1]/positive_limit for side in ['left_wing','right_wing'])-.99995
                    evaluated=(np.append(value['residual'],margin),value)
                    if frozen is not None:return evaluated
                    memo[key]=evaluated
                return memo[key]


            z=np.clip(np.array(seed['solution']+[seed['load_g']]),lower,upper)
            res,value=evaluate_at(z)
            if kind=='control':
                bounds_at_point=(self.instructor_at(value,speed_kmh/3.6,z[5])['control_bounds']
                    if self.config['instructor'] else value['allocation']['bounds'])
                z[axis]=np.clip(bounds_at_point[axis-2][1 if target>0 else 0]-target*3e-5,lower[axis],upper[axis])
                res,value=evaluate_at(z)
            if kind=='stall':
                z[0]=low['alpha_deg']+low['stall_margin_deg']-.01
                res,value=evaluate_at(z)
            coupled_derivatives=False
            for iteration in range(max_iterations):


                if (equilibrium_solver is self and self.is_prop and self.engine.automatic
                        and not self.engine.force_canonical
                        and (value['propulsion'].get('stationarity') or {}).get('method')=='bounded native limit-cycle mean'
                        and value['force_error_g']<.001 and max(abs(value['rate_residual']))<.001
                        and abs(res[5])<.001):
                    equilibrium_solver=self.with_prop_controls(value['propulsion']['controls'])
                    equilibrium_solver.engine.force_canonical=True
                    memo.clear();res,value=evaluate_at(z)


                force_goal=min(boundary_force_goal,1e-6) if z[5]<1.05 else boundary_force_goal
                rate_goal=min(boundary_rate_goal,2e-6) if z[5]<1.05 else boundary_rate_goal
                if value['force_error_g']<=force_goal and max(abs(value['rate_residual']))<=rate_goal and abs(res[5])<constraint_tolerance:break
                frozen=(value['propulsion'] if self.is_prop and self.engine.automatic and
                        (value['propulsion']['converged'] or self.engine.prefer_short_search) and
                        iteration<8 and not coupled_derivatives else None)
                derivative_res,derivative_value=evaluate_at(z,frozen) if frozen is not None else (res,value)
                cols=[]
                for i,h in enumerate([.002,.002,.0002,.0002,.0002,.001]):
                    direction=-1. if z[i]+h>upper[i] else 1.
                    for factor in [1.,.5,1.5,2.,-.5,-1.,.25,3.,.1,-.1,.05,-.05,.01,-.01]:
                        step=h*factor*direction;trial=z.copy();trial[i]+=step
                        if not lower[i]<=trial[i]<=upper[i]:continue
                        rr,vv=evaluate_at(trial,frozen);used_step=step
                        if self.derivative_branch(vv)==self.derivative_branch(derivative_value):break
                    cols.append((rr-derivative_res)/used_step)
                delta=np.linalg.lstsq(np.column_stack(cols),-res,rcond=None)[0]
                delta/=max(1.,float(np.max(abs(delta)/[5.,10.,.4,.4,.4,3.])))
                accepted=False
                factors=[1.,.5,.25,.1,.05]


                if self.is_prop and self.engine.automatic and (value['propulsion']['converged'] or self.engine.prefer_short_search):
                    phase=value['propulsion'];proxy_base,_=evaluate_at(z,phase)
                    offset=res-proxy_base
                    factors.sort(key=lambda factor:np.linalg.norm(
                        evaluate_at(np.clip(z+factor*delta,lower,upper),phase)[0]+offset))
                for factor in factors:
                    trial=np.clip(z+factor*delta,lower,upper);rr,vv=evaluate_at(trial)
                    if np.linalg.norm(rr)<np.linalg.norm(res):z,res,value=trial,rr,vv;accepted=True;break
                if not accepted:
                    if frozen is not None:coupled_derivatives=True;continue
                    break
            self.boundary_diagnostics.append(dict(kind=kind,state=z.tolist(),residual=res.tolist(),evaluations=len(memo)))
            if value['force_error_g']>2e-4 or max(abs(value['rate_residual']))>5e-5 or abs(res[5])>=constraint_tolerance:continue
            if z[5]<low['load_g']-.01:continue


            if high['converged'] and high['reasons'] and z[5]>high['load_g']+1e-8:continue
            if self.is_prop and self.engine.automatic and kind in ('stall','control','wing force','Instructor pitch'):


                from em_continuation import TrimCurve
                hint=dict(low,load_g=float(z[5]),solution=z[:5].tolist(),alpha_deg=float(z[0]),
                    _propulsion_seed=value['propulsion']['state'])
                if kind=='stall':
                    interior=TrimCurve(equilibrium_solver,speed_kmh,hint).limit(hint,'stall')
                else:
                    curve=TrimCurve(equilibrium_solver,speed_kmh,hint)
                    def active(v):
                        if kind=='wing force':return max(v['maximum_wing_load_ratios'])-.99995
                        if kind=='Instructor pitch':return self.instructor_at(v,speed_kmh/3.6,v['equilibrium_load_g'])['envelope_margin']-active_pitch_inset
                        bounds_at_point=(self.instructor_at(v,speed_kmh/3.6,v['equilibrium_load_g'])['control_bounds']
                            if self.config['instructor'] else v['allocation']['bounds'])
                        bound=bounds_at_point[axis-2][1 if target>0 else 0]
                        return v['equilibrium_coordinates'][axis]-(bound-target*3e-5)
                    interior=curve.correct(curve.state(hint),event=active,event_tolerance=constraint_tolerance,max_iterations=8)
                    if interior is not None and (abs(interior['alpha_deg']-hint['alpha_deg'])>1.5 or
                            max(abs(a-b) for a,b in zip(interior['solution'][2:],hint['solution'][2:]))>.2):
                        interior=None
                if interior is None:continue
            else:
                interior=equilibrium_solver.solve(speed_kmh,z[5],z[:5].tolist(),exhaustive=False,refine=compare_prop_controls or z[5]<1.05)
            if interior['valid']:
                if interior['load_g']<low['load_g']-.01:continue
                if high['converged'] and high['reasons'] and interior['load_g']>high['load_g']+1e-8:continue


                if kind=='stall':checked_margin=(interior['stall_margin_deg']-.002)/10.
                elif kind=='wing force':checked_margin=max(interior['wing_load_ratios'])-.99995
                elif kind=='control':
                    bound=interior['control_bounds'][axis-2][1 if target>0 else 0]
                    checked_margin=interior['solution'][axis]-(bound-target*3e-5)
                elif kind=='pitch response':checked_margin=interior['pitch_response']['margin']-response_inset
                else:checked_margin=interior['instructor']['envelope_margin']-active_pitch_inset
                if abs(checked_margin)>=constraint_tolerance:continue
                interior['envelope_limit']=dict(kind=kind,axis=axis if kind=='control' else None,
                                                limiting_load_g=interior['load_g'],constraint_residual=float(checked_margin),evaluations=len(memo))
                if kind=='Instructor pitch':
                    interior['envelope_limit']['pitch_inset']=active_pitch_inset
                if kind==crossed_kind and isolated_limit(interior,kind):return interior
                answers.append(interior)
        if not answers and self.config['structural_limits']:


            allowed={'wing force limit'}
            if low['valid'] and max(low['wing_load_ratios'])>.99995:
                inward=self.solve(speed_kmh,max(1.,low['load_g']-.005),low['solution'])
                if inward['valid']:low=inward
            if (low['converged'] and high['converged'] and
                set(low['reasons'])<=allowed and set(high['reasons'])<=allowed and
                max(low['wing_load_ratios'])<=.99995<max(high['wing_load_ratios'])):
                solved={low['load_g']:low,high['load_g']:high}
                class WingLimitClosed(Exception):
                    def __init__(self,point,error):self.point=point;self.error=error
                def wing_residual(n):
                    if n not in solved:
                        nearest=min(solved.values(),key=lambda p:abs(p['load_g']-n))
                        solved[n]=self.solve(speed_kmh,n,nearest['solution'])
                        if not solved[n]['converged']:
                            retry=self.solve(speed_kmh,n)
                            if retry['converged']:solved[n]=retry
                    p=solved[n]
                    if not p['converged'] or not set(p['reasons'])<=allowed:
                        raise ValueError('Numerical failure inside wing-limit bracket')
                    error=max(p['wing_load_ratios'])-.99995


                    if p['valid'] and abs(error)<5e-5:raise WingLimitClosed(p,error)
                    return error
                try:
                    n=brentq(wing_residual,low['load_g'],high['load_g'],xtol=1e-5,maxiter=24)
                    point=solved[n];error=wing_residual(n)
                    if point['valid'] and abs(error)<5e-5:
                        point['envelope_limit']=dict(kind='wing force',axis=None,limiting_load_g=n,
                            constraint_residual=error,evaluations=len(solved),method='balanced scalar refinement')
                        answers.append(point)
                except WingLimitClosed as closed:
                    point,error=closed.point,closed.error
                    point['envelope_limit']=dict(kind='wing force',axis=None,limiting_load_g=point['load_g'],
                        constraint_residual=error,evaluations=len(solved),method='balanced scalar refinement')
                    answers.append(point)
                except (ValueError,RuntimeError):pass
                if not answers:


                    for _ in range(18):
                        balanced_points=[p for p in solved.values() if p['converged'] and set(p['reasons'])<=allowed]
                        lower_points=[p for p in balanced_points if max(p['wing_load_ratios'])<=1.]
                        upper_points=[p for p in balanced_points if max(p['wing_load_ratios'])>1.]
                        if not lower_points or not upper_points:break
                        lower_point=max(lower_points,key=lambda p:p['load_g'])
                        upper_points=[p for p in upper_points if p['load_g']>lower_point['load_g']]
                        if not upper_points:break
                        upper_point=min(upper_points,key=lambda p:p['load_g'])
                        width=upper_point['load_g']-lower_point['load_g']
                        if width<.0005:
                            point=lower_point
                            point['envelope_limit']=dict(kind='wing force',axis=None,limiting_load_g=point['load_g'],
                                limiting_load_interval_g=[point['load_g'],upper_point['load_g']],
                                constraint_residual=max(point['wing_load_ratios'])-1.,evaluations=len(solved),
                                method='balanced physical constraint bracket')
                            answers.append(point);break
                        progressed=False
                        for fraction in [.5,.25,.75]:
                            n=lower_point['load_g']+fraction*width
                            try:wing_residual(n)
                            except WingLimitClosed as closed:
                                point,error=closed.point,closed.error
                                point['envelope_limit']=dict(kind='wing force',axis=None,limiting_load_g=point['load_g'],
                                    constraint_residual=error,evaluations=len(solved),method='balanced scalar refinement')
                                answers.append(point);break
                            except ValueError:continue
                            progressed=True;break
                        if answers or not progressed:break
        if not answers and near_level:
            from em_level_limit import near_level_boundary
            point=near_level_boundary(self,speed_kmh,low,high,candidate_kinds,max_iterations)
            if point:answers.append(point)
        if not answers and continuation and low['valid']:


            recovered=self.continue_boundary(speed_kmh,low,high)
            if recovered:answers.append(recovered)
        return min(answers,key=lambda p:p['load_g']) if answers else None

    def refine_permission_boundary(self,speed_kmh,low,high,strict=False):
        from em_constraint_bracket import refine
        return refine(self,speed_kmh,low,high)

    def continue_boundary(self,speed_kmh,low,high):
        point=low;step=.5
        lower=np.array([-15.,-1.,-1.,-1.,1.])
        from em_load_limits import ceiling
        upper=np.array([89.7,1.,1.,1.,ceiling(self,speed_kmh)])
        for iteration in range(64):
            alpha=point['alpha_deg']+step
            if alpha>self.trim_bounds[1][0]:return None
            memo={}
            def value(z):
                key=tuple(z)
                if key not in memo:memo[key]=self.operating_point(speed_kmh/3.6,z[-1],[alpha,*z[:4]])
                return memo[key]
            def fun(z):return value(z)['residual']
            def jac(z):
                base=value(z);cols=[]
                for i,h in enumerate([.002,.0002,.0002,.0002,.001]):
                    for factor in [1.,.5,-.5,2.,-1.,.1,-.1,.01,-.01]:
                        trial=z.copy();trial[i]+=h*factor;v=value(trial)
                        if self.derivative_branch(v)==self.derivative_branch(base):break
                    cols.append((v['residual']-base['residual'])/(h*factor))
                return np.column_stack(cols)
            z=np.array(point['solution'][1:]+[point['load_g']])
            z=np.clip(z,lower+1e-8,upper-1e-8)
            fit=least_squares(fun,z,jac=jac,bounds=(lower,upper),x_scale=[30.,.2,.2,.2,5.],
                              max_nfev=32,ftol=1e-9,xtol=2e-7,gtol=1e-8)
            v=value(fit.x)
            balanced=v['force_error_g']<=2e-4 and max(abs(v['rate_residual']))<=5e-5 and v['history_error']<=2e-4
            if balanced:
                trial=self.solve(speed_kmh,fit.x[-1],[alpha,*fit.x[:4]],exhaustive=False)
                if trial['load_g']<point['load_g']-.0002:return None
                if not trial['valid'] or trial['authority_margin']<.08 or trial['stall_margin_deg']<.15 or (
                    self.config['structural_limits'] and max(trial['wing_load_ratios'])>.98) or (
                    self.config['instructor'] and trial['instructor']['margin']<.01):
                    limit=self.boundary(speed_kmh,point,trial,continuation=False)
                    if limit:
                        limit['envelope_limit']['seed_method']='balanced angle continuation'
                        limit['envelope_limit']['continuation_steps']=iteration+1
                        return limit
                if trial['valid']:
                    point=trial;step=min(.5,step*1.5);continue


            step*=.5
            if step<.0078125:
                limit=self.boundary(speed_kmh,point,high,continuation=False)
                if limit:
                    limit['envelope_limit']['seed_method']='balanced angle continuation'
                    limit['envelope_limit']['continuation_steps']=iteration+1
                return limit
        return None

    from em_operating import complete_reporting

    def operating_point(self, speed, load, x, *args, **kwargs):
        from em_operating import operating_point
        from em_trim_work import charge, observe
        charge('aircraft_evaluations')
        value=operating_point(self, speed, load, x, *args, **kwargs)
        frozen=kwargs.get('propulsion_override') is not None or bool(args and args[0] is not None)
        observe(self, speed, load, x, value, frozen)
        return value
    _defer_search_reporting=True

    def initial_guess(self,speed,load):
        air=cache([f32(speed),0.,0.],self.config['altitude_m'])
        flaps=self.flaps
        polar=condition_properties(self.model,air['mach'],flaps)[1]
        scale=.5*air['density']*speed*speed*self.model['geometry']['area']
        cl=load*self.weight/max(scale,1.)
        slope=polar['clLineCoeff']
        alpha=(cl-polar['cl0'])/slope if abs(slope)>1e-8 else polar['aoaLineH']
        incidence=self.model['geometry']['incidence']
        alpha-=incidence
        alpha=float(np.clip(alpha,self.trim_bounds[0][0]+1.,
                            min(self.trim_bounds[1][0]-1.,polar['aoaCritH']-incidence-2.)))
        return [alpha,math.degrees(math.acos(1/load)),0.,0.,0.]

    def certify(self, speed_kmh, load, x, final, *, evaluations=0, detailed=False,
                bound_stationary=False):
        speed=speed_kmh/3.6
        x=np.asarray(x)
        self.complete_reporting(final)
        aero=final['result']; geom=final['geometry']; strength=self.model['geometry']['strength']
        ratios=final['maximum_wing_load_ratios']
        finite=np.isfinite(np.r_[x,final['residual'],final['rate_residual'],aero['force'],aero['stored_moment'],
            final['force_mean_uncertainty_g'],final['angular_mean_uncertainty_rad_s2'],final['history_error'],
            final['stall_margin'],final['ps'],ratios,final['kinematic']['velocity'],aero['air']['ias_u'],aero['air']['mach']]).all()
        converged=(finite and final['force_error_g']+final['force_mean_uncertainty_g']<=2e-4 and
                   max(abs(final['rate_residual'])+final['angular_mean_uncertainty_rad_s2'])<=5e-5 and final['history_error']<=2e-4)
        reasons=[]
        from em_load_limits import ceiling
        if getattr(self,'chart_search',False) and load>ceiling(self,speed_kmh)+1e-10:
            reasons.append('chart load search limit')
        if not converged:reasons.append('trim did not converge')
        if self.is_prop:
            propulsion=final['propulsion']
            if not propulsion['converged']:reasons.append('propulsion did not settle')


            if propulsion['stopped_shafts']:reasons.append('propeller shaft stopped')
            phases=propulsion.get('cycle_samples')
            phase_check=dict(checked=False,force_error_g=None,angular_error_rad_s2=None,history_error=None)
            stationary_mean=propulsion.get('stationarity',{}).get('method')=='bounded native limit-cycle mean' if propulsion.get('stationarity') else False


            if not phases or final.get('propulsion_phase_frames',0)!=len(phases):
                reasons.append('propulsion cycle unresolved')
            elif converged:
                phase_check=dict(checked=True,model=(('Quasi-steady ideal-governor aircraft equilibrium')),
                    force_error_g=final['force_error_g'],angular_error_rad_s2=float(max(abs(final['rate_residual']))),
                    force_mean_uncertainty_g=final['force_mean_uncertainty_g'],
                    angular_mean_uncertainty_rad_s2=final['angular_mean_uncertainty_rad_s2'].tolist(),
                    history_error=final['history_error'],periodic_aircraft_trajectory_certified=False)
        if not self.config['instructor'] and not final['allocation']['reachable']:reasons.append('control authority')
        if final['stall_margin']<0:reasons.append('post-stall')
        if abs(final['kinematic']['velocity'][1])>.005:
            reasons.append('vertical step did not close')
        structural=[]
        if max(ratios)>1:structural.append('wing force limit')
        if aero['air']['ias_u']>strength['ias']:structural.append('IAS limit')
        if aero['air']['mach']>strength['mach']:structural.append('Mach limit')
        sweep_range=available_sweep(self.fm,self.sweep_rows,aero['air']['mach']) if self.has_sweep else (0.,1.)
        if self.has_sweep and not sweep_range[0]-1e-7<=self.model['sweep']<=sweep_range[1]+1e-7:reasons.append('sweep unavailable')
        if self.config['structural_limits']:reasons+=structural
        from flap_model import violations as flap_violations
        reasons+=flap_violations(self.flap_profile,self.config['flaps_percent'],
            aero['air']['ias_u'],aero['air']['mach'],self.config['structural_limits'])
        pitch=None
        if converged and final['stall_margin']>=0. and not reasons:
            pitch=pitch_response(self,final)
            if not pitch['normal']:
                reasons.append('reversed pitch response' if pitch['reversed'] else 'pitch response unresolved')
        instructor=None
        if self.config['instructor'] and not reasons:


            instructor=self.instructor_at(final,speed,load)
            if not instructor['converged']:reasons.append('Instructor boundary unresolved')
            else:
                if instructor['envelope_margin']<-2e-7:reasons.append('Instructor pitch limit')
                if instructor['margins']['control authority with auto trim']<-2e-7:reasons.append('control authority')
        control_bounds=instructor['control_bounds'] if instructor is not None else final['allocation']['bounds']
        point=dict(speed_kmh=float(speed_kmh),load_g=float(load),turn_dps=math.degrees(geom['turn_rate']),
                   aircraft_trim_mode=self.config['aircraft_trim_mode'],ps_definition='discrete energy step',
                   ps_mps=final['ps'],ps_continuous_mps=final['ps_continuous'],alpha_deg=float(x[0]),bank_deg=float(x[1]),
                   sideslip_deg=float(aero['air']['beta']),sideslip_attitude_deg=self.sideslip_attitude_deg,
                   roll_leveling_branch=(-1 if aero['air']['alpha']<=-12. else 1 if aero['air']['alpha']>=12. else 0) if self.fm.get('RollLeveling',True) else None,
                   converged=bool(converged),valid=not reasons,reasons=reasons,structural_flags=structural,
                   bounded_search_stationary=bool(bound_stationary),
                   pitch_response=pitch,
                   force_error_g=float(final['force_error_g']),angular_error_rad_s2=float(max(abs(final['rate_residual']))),
                   history_error=float(final['history_error']),stall_margin_deg=float(final['stall_margin']),
                   negative_stall_margin_deg=float(final['negative_stall_margin']),
                   wing_load_ratios=ratios,commands=final['allocation']['commands'],
                   control_bounds=control_bounds,authority_margin=min(min(d-lo,hi-d) for d,(lo,hi) in zip(final['allocation']['commands'],control_bounds)),
                   sticks=instructor.get('sticks',final['allocation']['sticks']) if instructor else final['allocation']['sticks'],
                   trim=instructor['auto_trim'] if instructor else final['allocation']['trim'],
                   ias_kmh=aero['air']['ias_u']*3.6,mach=aero['air']['mach'],
                   sweep_percent=self.config['sweep_percent'] if self.has_sweep else None,
                   flaps_percent=final['flaps']*100.,flaps_requested_percent=self.config['flaps_percent'],
                   gear_percent=self.gear*100.,
                   instructor=final['instructor'],instructor_enabled=self.config['instructor'],
                   sweep_available_percent=[x*100 for x in sweep_range] if self.has_sweep else None,
                   force_n=aero['force'],moment_nm=aero['stored_moment'],engine_force_n=aero['engine_force'],
                   component_forces=aero['component_forces'],evaluations=evaluations,solution=x.tolist(),
                   vertical_step_velocity_mps=final['kinematic']['velocity'][1],
                   altitude_correction=final['altitude_correction'],
                   actual_turn_dps=math.degrees(math.atan2(final['kinematic']['velocity'][2],final['kinematic']['velocity'][0])/self.dt))
        pass
        if self.is_prop:
            p=final['propulsion']
            if converged and p['converged'] and phase_check['checked'] and self.engine.automatic:


                from em_data import clone
                point['_propulsion_seed']=clone(p['state'])
            point['propulsion']=dict(controls=p['controls'],converged=p['converged'],
                canonical_initialization=p['canonical_initialization'],
                period_frames=p['period_frames'],window_relative_change=p['window_relative_change'],
                stationarity=p.get('stationarity'),sample_frames=len(p.get('cycle_samples') or []),
                phase_check=phase_check,
                engine_rpm=[e['omega']*60/(2*math.pi) for e in p['state']['engines']],
                rpm_threshold_exceeded_engines=p['overspeed_engines'],
                engine_rpm_threshold=[e['properties']['omega_limit']*60/(2*math.pi) for e in self.engine.properties['engines']],
                compressor_stages=[e.get('gear',0) for e in p['state']['engines']],
                engine_control_mode=p['controls'].get('engine_control_mode','optimized'),
                mixture=[e.get('mixture') for e in p['state']['engines']],
                shaft_power_kw=[e.get('torque',0.)*e['omega']/1000. for e in p['state']['engines']],
                propeller_pitch_deg=[e['pitch']*180/math.pi for e in p['state']['propellers']],
                wash_mps=p['wash'],angular_momentum=p['angular_momentum'])
        if self.is_prop and point['propulsion']['engine_control_mode']=='automatic':
            point['propulsion']['optimization']=dict(method='Dynamic propulsion with automatic controls',evaluated=0,
                global_optimum_certified=False,radiators_closed=True)
        elif self.is_prop and self.engine.quasi_steady:
            point['propulsion']['optimization']=dict(method='Quasi-steady ideal RPM governor',evaluated=0,
                global_optimum_certified=False,radiators_closed=True)
        if detailed:point['_detail']=final
        from em_trim_work import record_point
        record_point(point)
        return point

    def solve(self, speed_kmh, load, initial=None, detailed=False, exhaustive=True, refine=False,quick=False):
        from em_load_limits import excluded_point
        excluded=excluded_point(self,speed_kmh,load,initial)
        if excluded is not None:return excluded
        from em_trim_search import solve
        return solve(self, speed_kmh, load, initial, detailed, exhaustive, refine, quick)

    def _solve_attempt(self, speed_kmh, load, initial=None, detailed=False, exhaustive=True, refine=False,quick=False):
        from em_load_limits import excluded_point
        excluded=excluded_point(self,speed_kmh,load,initial)
        if excluded is not None:return excluded
        from em_trim_search import Request
        pass
        seeded=initial is not None
        speed=speed_kmh/3.6; initial=initial or self.initial_guess(speed,load)


        defer_reporting=self._defer_search_reporting and (not self.is_prop or self.engine.quasi_steady)
        memo={}; calls=0;last_jacobian=None
        def evaluate_x(x):
            nonlocal calls
            check_cancel()
            key=tuple(x)
            if key not in memo:
                if len(memo)>256:memo.clear()


                budget=(20. if quick or self.engine.prefer_short_search else 60.) if self.is_prop and self.engine.automatic else 60.
                memo[key]=self.operating_point(speed,load,x,cycle_seconds=budget,_search=defer_reporting); calls+=1
            return memo[key]
        def fun(x):return evaluate_x(x)['residual']
        def jac(x,freeze_propulsion=False):
            nonlocal last_jacobian
            base=evaluate_x(x); steps=[.002,.002,.0002,.0002,.0002];columns=[]
            derivative_base=(self.operating_point(speed,load,x,propulsion_override=base['propulsion'],_search=defer_reporting)
                             if freeze_propulsion else base)


            from em_trim_numerics import jacobian
            from em_mach_events import rough_mach,project_state
            mach=derivative_base['result']['air']['mach']
            project=(lambda q:project_state(self,speed,load,q,mach)) if rough_mach(self,mach) else None
            sample=(lambda q:self.operating_point(speed,load,q,propulsion_override=base['propulsion'],_search=defer_reporting)) if freeze_propulsion else evaluate_x
            last_jacobian=jacobian(x,derivative_base,sample,self.derivative_branch,steps,
                [1.,.5,1.5,2.,.25,-.5,-1.,-1.5,-2.,.1,3.,4.,5.,-.1,.05,-.05,.01,-.01],
                project=project,actual_step=project is not None)
            return last_jacobian
        bounds=self.trim_bounds
        x0=np.clip(initial,np.asarray(bounds[0])+1e-6,np.asarray(bounds[1])-1e-6)
        bound_stationary=False
        def direction(matrix,residual,x,scale):
            scale=np.asarray(scale);delta=np.linalg.lstsq(matrix,-residual,rcond=None)[0]
            delta/=max(1.,float(np.max(abs(delta)/scale)))
            lo=np.maximum(np.asarray(bounds[0])-x,-scale)
            hi=np.minimum(np.asarray(bounds[1])-x,scale)
            if np.any(delta<lo) or np.any(delta>hi):
                fit=lsq_linear(matrix*scale,-residual,bounds=(lo/scale,hi/scale),
                               method='bvls',tol=1e-10,max_iter=20)
                delta=fit.x*scale
            return delta
        def balanced(value):


            return (np.isfinite(value['residual']).all() and np.isfinite(value['rate_residual']).all() and
                    value['force_error_g']<=(1e-6 if refine else 2e-4) and
                    max(abs(value['rate_residual']))<=(2e-6 if refine else 5e-5) and
                    value['force_error_g']+value['force_mean_uncertainty_g']<=2e-4 and
                    max(abs(value['rate_residual'])+value['angular_mean_uncertainty_rad_s2'])<=5e-5)
        class ClosedEquilibrium(Exception):
            def __init__(self,x,value):self.x=np.array(x);self.value=value
        def fit_equilibrium(_fun,x,**options):


            def checked(q):
                v=evaluate_x(q)
                if balanced(v) and v['history_error']<=2e-4:
                    raise ClosedEquilibrium(q,v)
                return v['residual']
            try:return least_squares(checked,x,**options)
            except ClosedEquilibrium as closed:
                return SimpleNamespace(x=closed.x,fun=closed.value['residual'])


        x=x0.copy(); value=evaluate_x(x)
        from em_mach_events import rough_mach,project_state
        def native_proposal(q,base):
            mach=base['result']['air']['mach']
            return project_state(self,speed,load,q,mach) if rough_mach(self,mach) else q
        prop_blocked=(self.is_prop and getattr(self.engine,'no_resolved_seed',False)
                      and not value['propulsion']['converged'])
        predictor=getattr(self,'_trim_predictor',None)
        if (predictor and predictor[0]==(speed_kmh,self.sideslip_attitude_deg) and not prop_blocked
                and abs(math.sqrt(max(0.,load*load-1.))-math.sqrt(max(0.,predictor[2]**2-1.)))
                    <=.25*max(1.,min(math.sqrt(max(0.,load*load-1.)),math.sqrt(max(0.,predictor[2]**2-1.))))):
            matrix=predictor[1].copy()
            for _ in range(4):
                if balanced(value):break
                delta=np.linalg.lstsq(matrix,-value['residual'],rcond=None)[0]
                delta/=max(1.,float(np.max(abs(delta)/[3.,10.,.2,.2,.2])))
                advanced=False
                for factor in (1.,.5,.25):
                    q=native_proposal(np.clip(x+factor*delta,*bounds),value);v=evaluate_x(q)
                    if balanced(v) or np.linalg.norm(v['residual'])<np.linalg.norm(value['residual']):
                        dx=q-x;den=float(dx.dot(dx))
                        if den>1e-12:
                            matrix+=np.outer(v['residual']-value['residual']-matrix.dot(dx),dx)/den
                        x,value=q,v;advanced=True;break
                if not advanced:break
            if balanced(value):last_jacobian=matrix
        for iteration in range(0 if prop_blocked else 8):
            if balanced(value):break


            fast_jacobian=self.is_prop and self.engine.automatic
            if fast_jacobian:


                frozen=value['propulsion'];inner=x.copy()
                proxy=self.operating_point(speed,load,inner,propulsion_override=frozen,_search=defer_reporting)
                offset=value['residual']-proxy['residual'];res=proxy['residual']+offset
                for _ in range(5):
                    from em_trim_numerics import jacobian
                    last_jacobian=jacobian(inner,proxy,
                        lambda q:self.operating_point(speed,load,q,propulsion_override=frozen,_search=defer_reporting),
                        self.derivative_branch,[.002,.002,.0002,.0002,.0002],
                        [1.,.5,2.,-.5,-1.,.1,-.1,.01,-.01])
                    correction=direction(last_jacobian,res,inner,[8.,20.,.4,.4,.4])
                    advanced=False
                    for factor in [1.,.5,.25,.1]:
                        trial=np.clip(inner+factor*correction,*bounds)
                        vv=self.operating_point(speed,load,trial,propulsion_override=frozen,_search=defer_reporting)
                        rr=vv['residual']+offset
                        if np.linalg.norm(rr)<np.linalg.norm(res):
                            inner,proxy,res=trial,vv,rr;advanced=True;break
                    if not advanced or np.max(abs(res))<1e-6:break
                delta=inner-x
            else:delta=direction(jac(x),value['residual'],x,[8.,20.,.4,.4,.4])
            at_control_stop=any(abs(x[i]-bounds[j][i])<1e-6 for i in (2,3,4) for j in (0,1))
            if at_control_stop and np.max(abs(delta)/[8.,20.,.4,.4,.4])<1e-6:
                bound_stationary=True;break
            scale=max(1.,float(np.max(np.abs(delta)/[8.,20.,.4,.4,.4])))
            delta/=scale
            norm=np.linalg.norm(value['residual']);accepted=False
            for factor in [1.,.5,.25,.1]:
                trial_x=native_proposal(np.clip(x+delta*factor,*bounds),value);trial=evaluate_x(trial_x)
                if balanced(trial) or np.linalg.norm(trial['residual'])<norm:
                    x,value=trial_x,trial;accepted=True;break
            if not accepted:break
        if (not quick and not prop_blocked and not bound_stationary and self.is_prop and self.engine.automatic
                and not self.engine.force_canonical and not balanced(value)
                and value['propulsion'].get('stationarity')
                and value['propulsion']['converged'] and value['stall_margin']>0.
                and value['force_error_g']<.015 and max(abs(value['rate_residual']))<.008):
            strict=self.with_prop_controls(value['propulsion']['controls'])
            strict.engine.force_canonical=True
            point=(yield Request(strict, speed_kmh,load,x.tolist(),detailed=detailed,
                               exhaustive=False,refine=refine))
            if point['valid']:
                point['evaluations']+=calls
                return point
        if not quick and not prop_blocked and not bound_stationary and not balanced(value) and self.is_prop and self.engine.automatic:


            fit=fit_equilibrium(fun,x,jac=lambda q:jac(q,True),bounds=bounds,
                x_scale=[10.,30.,.2,.2,.2],max_nfev=10,ftol=1e-9,xtol=2e-7,gtol=1e-8)
            trial=evaluate_x(fit.x)
            if balanced(trial) or np.linalg.norm(trial['residual'])<np.linalg.norm(value['residual']):
                x,value=fit.x,trial
        result=SimpleNamespace(x=x,fun=value['residual']) if prop_blocked or bound_stationary or balanced(value) or quick else fit_equilibrium(
            fun,x,jac=jac,bounds=bounds,x_scale=[10.,30.,.2,.2,.2],
            max_nfev=45 if exhaustive else 10,ftol=1e-9,xtol=2e-7,gtol=1e-8)
        final=evaluate_x(result.x)
        if exhaustive and not prop_blocked and not bound_stationary and (not balanced(final) or final['stall_margin']<0):


            fresh=np.clip(self.initial_guess(speed,load),
                          np.asarray(bounds[0])+1e-6,np.asarray(bounds[1])-1e-6)
            retry=fit_equilibrium(fun,fresh,jac=jac,bounds=bounds,x_scale=[10.,30.,.2,.2,.2],
                                max_nfev=45,ftol=1e-9,xtol=2e-7,gtol=1e-8)
            retry_final=evaluate_x(retry.x)
            prefer_branch=balanced(retry_final) and retry_final['stall_margin']>=0 and (not balanced(final) or final['stall_margin']<0)
            same_branch=(retry_final['stall_margin']>=0)==(final['stall_margin']>=0)
            if prefer_branch or (same_branch and np.linalg.norm(retry.fun)<np.linalg.norm(result.fun)):
                result=retry;final=retry_final
        if exhaustive and not prop_blocked and not bound_stationary and not balanced(final) and final['force_error_g']<.05 and max(abs(final['rate_residual']))<.02:


            best_x=result.x.copy();best=final
            for bank_offset in [0.,.0001,-.0001,.0005,-.0005]:
                x=best_x.copy();x[1]+=bank_offset;x=np.clip(x,*bounds);value=evaluate_x(x)
                for iteration in range(12):
                    if balanced(value):break
                    delta=np.linalg.lstsq(jac(x),-value['residual'],rcond=None)[0]
                    candidate_x=x;candidate=value
                    for factor in [1.]+np.linspace(.05,1.6,63).tolist():
                        trial_x=np.clip(x+factor*delta,*bounds);trial=evaluate_x(trial_x)
                        if np.linalg.norm(trial['residual'])<np.linalg.norm(candidate['residual']) or balanced(trial):
                            candidate_x,candidate=trial_x,trial
                        if balanced(trial):break
                    if np.array_equal(candidate_x,x):break
                    x,value=candidate_x,candidate
                if balanced(value) or np.linalg.norm(value['residual'])<np.linalg.norm(best['residual']):best_x,best=x,value
                if balanced(best):break
            result.x=best_x;final=best
        if self.is_prop and not self.engine.force_canonical and (not quick or balanced(final)):


            canonical=((self.operating_point(speed,load,result.x,cycle_seconds=20.,phase_certificate=True)))
            if self.engine.automatic and not canonical['propulsion']['converged']:


                canonical=self.operating_point(speed,load,result.x,cycle_seconds=180.,phase_certificate=True)
            if self.engine.automatic and not canonical['propulsion']['converged']:
                canonical=self.operating_point(speed,load,result.x,canonical_propulsion=True)


            pass


            near_closure=(canonical['propulsion']['converged'] and
                          canonical['force_error_g']<.015 and max(abs(canonical['rate_residual']))<.008)
            if not bound_stationary and not balanced(canonical) and (balanced(final) or near_closure):
                strict=self.with_prop_controls(canonical['propulsion']['controls'])


                strict.engine.force_canonical=True
                point=(yield Request(strict, speed_kmh,load,result.x.tolist(),detailed=detailed,exhaustive=exhaustive,refine=refine,quick=quick))
                point['evaluations']+=calls;return point
            final=canonical
        point=self.certify(speed_kmh,load,result.x,final,evaluations=calls,
                           detailed=detailed,bound_stationary=bound_stationary)
        pitch=point['pitch_response']
        if pitch and pitch['reversed']:
            recovered=(yield from recover_pitch_response(self,final,detailed=detailed,refine=refine))
            if recovered is not None:
                recovered['evaluations']+=calls
                return recovered
        converged=point['converged'];reasons=point['reasons']
        if (self.config['instructor'] and not refine and not getattr(self,'_instructor_rounding_search',False) and point['converged']
                and point['reasons']==['Instructor pitch limit']
                and point['instructor'].get('pitch_predictor_recheck',True)
                and point['instructor']['pitch_margin']<-2e-7):


            corrected=(yield Request(self, speed_kmh,load,point['solution'],detailed=True,
                                 exhaustive=False,refine=True,quick=True))
            if (not corrected['valid'] and point['instructor']['angle_margin']>.05
                    and point['authority_margin']>.01
                    and abs(point['instructor']['delivered_pitch']-point['instructor']['auto_trim'][1])<1e-7):


                import copy
                original_corrected=corrected
                for origin,condition in ((point,final),(original_corrected,original_corrected['_detail'])):
                    nearby=copy.copy(self)
                    if self.is_prop:
                        nearby=self.with_prop_controls(origin['propulsion']['controls'])
                        nearby.engine._reference=copy.deepcopy(condition['propulsion']['state'])
                        nearby.engine.force_canonical=True
                    nearby._instructor_rounding_search=True


                    ctl=origin.get('instructor') or point['instructor']
                    collapsed=abs(ctl['delivered_pitch']-ctl['auto_trim'][1])<1e-7
                    for scale in ((8.,64.,512.) if collapsed else (8.,)):
                        for axis in (3,0,1):
                            step=scale*max(abs(float(np.spacing(np.float32(origin['solution'][axis])))),1e-10)
                            for sign in (-1.,1.):
                                guess=list(origin['solution']);guess[axis]+=sign*step
                                trial=(yield Request(nearby, speed_kmh,load,guess,detailed=detailed,exhaustive=False,quick=True))
                                if trial['valid']:corrected=trial;break
                            if corrected['valid']:break
                        if corrected['valid']:break
                    if corrected['valid']:break
                if (not corrected['valid'] and self.is_prop and self.engine.automatic
                        and not getattr(self,'_instructor_independent_recheck',False)):


                    independent=self.with_prop_controls(point['propulsion']['controls'])
                    independent.engine._warm=None;independent.engine._reference=None
                    independent.engine._task_seed=None;independent.engine.force_canonical=False
                    independent.engine.prefer_short_search=False;independent.engine.search_cycle_seconds=20.


                    independent.instructor_trim_entries=self.instructor_trim_entries
                    independent._instructor_independent_recheck=True
                    trial=(yield Request(independent, speed_kmh,load,point['solution'],
                        detailed=detailed,exhaustive=False,quick=True))
                    if trial['valid']:
                        corrected=trial
                        corrected['instructor_independent_restart']=True
            if corrected['valid']:
                if not detailed:corrected.pop('_detail',None)
                corrected['evaluations']+=calls
                corrected['instructor_recheck']=dict(method='balanced trim and native rounding-branch recheck before Instructor exclusion',
                    original_force_error_g=point['force_error_g'],
                    original_angular_error_rad_s2=point['angular_error_rad_s2'],
                    original_pitch_margin=point['instructor']['pitch_margin'],
                    original_envelope_margin=point['instructor']['envelope_margin'])
                return corrected
        if point['valid'] and last_jacobian is not None:


            self._trim_predictor=((speed_kmh,self.sideslip_attitude_deg),last_jacobian,load)
        if detailed:point['_detail']=final
        if exhaustive and not prop_blocked and not bound_stationary and seeded and (not converged or 'post-stall' in reasons):


            independent=(yield Request(self, speed_kmh,load,detailed=detailed,exhaustive=True))
            if independent['valid']:
                independent['evaluations']+=calls
                independent['recovery']='independent equilibrium restart'
                return independent
        if exhaustive and not prop_blocked and not bound_stationary and not converged and final['force_error_g']<.02 and max(abs(final['rate_residual']))<.01:


            for alpha_offset in [.08,-.08,.3,-.3]:
                guess=result.x.copy();guess[0]+=alpha_offset
                retry=(yield Request(self, speed_kmh,load,guess.tolist(),detailed=detailed,exhaustive=False))
                if retry['valid']:
                    retry['evaluations']+=calls
                    retry['recovery']='independent local branch restart'
                    return retry
        return point


def compute(config=None, progress=None, cancelled=None, preview=None):
    config=settings(config)
    if config['entries'] is not None:
        from em_entries import compute_entries
        return compute_entries(config,progress,cancelled,preview)
    if config['compare_instructor']:
        start=time.monotonic();name=config['aircraft'][0];runs=[]
        for index,enabled in enumerate((False,True)):
            if cancelled and cancelled():raise InterruptedError('Calculation cancelled')
            condition=aircraft_settings(config,name)
            condition.update(compare_instructor=False,instructor=enabled)
            def report(p):
                total=p.get('total',0)
                progress(dict(p,done=index*total+p.get('done',0),total=2*total,
                              phase=f"Instructor {'on' if enabled else 'off'} · "+p.get('phase','Solving'),
                              elapsed_s=time.monotonic()-start))
            run=compute(condition,report if progress else None,cancelled)
            aircraft=run['aircraft'][0]
            aircraft.update(id=name+('__instructor_on' if enabled else '__instructor_off'),aircraft_id=name,
                            name=AIRCRAFT[name]['name']+(' · Instructor on (experimental)' if enabled else ' · Instructor off'),
                            color=['#38c9d7','#ffa66b'][index])
            runs.append(run)
        output=dict(runs[0],settings=config,aircraft=[r['aircraft'][0] for r in runs],
                    speeds_kmh=sorted(set(v for r in runs for v in r['speeds_kmh'])),
                    elapsed_s=time.monotonic()-start)
        output['assumptions']=[a for a in output['assumptions'] if a!='Instructor off']
        output['assumptions'].append('Same flight conditions with Instructor off and experimental Instructor on')
        return output
    if config['sampling']=='adaptive':
        from em_sampling import compute_cached
        return compute_cached(config,progress,cancelled,preview)
    return compute_regular(config,progress,cancelled)


def compute_regular(config=None, progress=None, cancelled=None):
    config=settings(config); start=time.monotonic()
    speeds=np.linspace(config['speed_min_kmh'],config['speed_max_kmh'],config['speed_samples']).tolist()
    loads=np.linspace(1.,config['max_load_g'],config['load_samples']).tolist()
    output=dict(settings=config,speeds_kmh=speeds,loads_g=loads,aircraft=[],method='coordinated trim below positive stall; per-aircraft flight model and Ps definition',
                assumptions=['Full-real manual aerodynamic trim','Full pilot authority; aerodynamic control-power loss retained',
                             'Constant fuel and intact components','Positive-AoA stall enforced; negative-AoA stall not an exclusion; aircraft trim model selected per entry','Still air; out of ground effect; retracted gear/brake',
                             'Fixed flap extension; speed domain ends at the selected extension’s automatic IAS/Mach limit or intact-flap damage threshold; flap travel and damage transients omitted',
                             'Steady Instructor AoA schedule approximation; transient overshoot, delay and control history omitted' if config['instructor'] else 'Instructor off',
                             'Manual additional mass is at configured CG; default ammunition uses native per-weapon masses and attachment positions'],
                validation='Reconstructed kernels have native-code comparisons; this EM solver has not been validated against live flight.')
    total=len(speeds)*len(loads)*len(config['aircraft']); done=0
    for index,name in enumerate(config['aircraft']):
        solver=TrimSolver(name,config);solver.chart_search=True; columns=[]
        from em_speed_limits import excluded_point,speed_limits
        for i,speed in enumerate(speeds):
            column=[]
            for j,load in enumerate(loads):
                if cancelled and cancelled():raise InterruptedError('Calculation cancelled')
                initial=None
                if j and column[-1]['converged']:initial=column[-1]['solution']
                elif i and columns[-1][j]['converged']:initial=columns[-1][j]['solution']
                point=excluded_point(solver,speed,load)
                if point is None:point=solver.solve(speed,load,initial)
                column.append(point); done+=1
                if progress:progress(dict(done=done,total=total,aircraft=name,speed_kmh=speed,load_g=load,
                                          valid=point['valid'],elapsed_s=time.monotonic()-start))
            columns.append(column)
        sustained=[]
        for i,column in enumerate(columns):
            for low,high in zip(column,column[1:]):


                if low['valid'] and low['ps_mps']>0 and not high['valid']:
                    left,right=low,high
                    for attempt in range(8):
                        if cancelled and cancelled():raise InterruptedError('Calculation cancelled')
                        n=(left['load_g']+right['load_g'])*.5
                        middle=solver.solve(speeds[i],n,left['solution'])
                        if middle['valid'] and middle['ps_mps']<=0:low,high=left,middle;break
                        if middle['valid']:left=middle
                        else:right=middle
                if not (low['valid'] and high['valid'] and low['ps_mps']*high['ps_mps']<=0):continue
                if cancelled and cancelled():raise InterruptedError('Calculation cancelled')
                if progress:progress(dict(done=done,total=total,aircraft=name,speed_kmh=speeds[i],phase='Refining Ps = 0',elapsed_s=time.monotonic()-start))
                solved={low['load_g']:low,high['load_g']:high}
                def root_function(n):
                    if cancelled and cancelled():raise InterruptedError('Calculation cancelled')
                    if n not in solved:
                        mix=(n-low['load_g'])/(high['load_g']-low['load_g'])
                        initial=(np.asarray(low['solution'])*(1-mix)+np.asarray(high['solution'])*mix).tolist()
                        solved[n]=solver.solve(speeds[i],n,initial)
                    if not solved[n]['valid']:raise ValueError('Invalid root bracket interior')
                    return solved[n]['ps_mps']
                try:
                    n=brentq(root_function,low['load_g'],high['load_g'],xtol=2e-5,maxiter=18)
                    point=solved[n]
                    if abs(point['ps_mps'])<.02:sustained.append(point)
                except (ValueError,RuntimeError):pass
        points=[columns[i][j] for j in range(len(loads)) for i in range(len(speeds))]
        valid=[p for p in points if p['valid']]
        metadata=dict(AIRCRAFT[name],color=['#38c9d7','#ffa66b'][index])
        from aircraft_description import description
        output['aircraft'].append(dict(id=name,**metadata,settings=solver.config,aircraft_model=description(solver.config),mass=solver.mass,engine=solver.engine.summary,points=points,
                                       speed_limit=speed_limits(solver.fm,solver.config),
                                       instructor_approximation=instructor_profile(solver.fm,solver.config) if solver.config['instructor'] else None,
                                       sustained=sustained,valid_points=len(valid),converged_points=sum(p['converged'] for p in points)))
    output['elapsed_s']=time.monotonic()-start
    return output


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',default='outputs/em-sample.json'); parser.add_argument('--config')
    args=parser.parse_args(); config=json.loads(Path(args.config).read_text()) if args.config else dict(speed_samples=9,load_samples=7)
    def progress(p):
        if p['done']%10==0:print(f"{p['done']}/{p['total']} · {p['aircraft']} · {p['speed_kmh']:.0f} km/h",flush=True)
    result=compute(config,progress)
    path=Path(args.output);path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(path,round(result['elapsed_s'],1),'seconds')
