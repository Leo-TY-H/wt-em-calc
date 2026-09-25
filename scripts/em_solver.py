"""Coordinated EM slices below positive stall using the recovered aircraft equations.

This is a numerical trim solver, not a live-game performance oracle. Ps is the
native finite-step energy change; nonzero-Ps points are accelerating trimmed
operating points, not sustained trajectories. See analysis/em-method.md.
"""
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
from em_pitch_response import response as pitch_response, recover as recover_pitch_response, KINDS as PHYSICAL_KINDS

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
                afterburner=True, torque_gyro=False, engine_control_mode='automatic', trim_mode='optimized', trim_limit=1., fixed_trim=[0., 0., 0.],
                extra_mass_kg=0., speed_min_kmh=100., speed_max_kmh=1300., max_load_g=None,
                speed_samples=9, load_samples=9, structural_limits=True, timestep_hz=48.,
                sampling='adaptive',sep_tolerance_mps=.5,surface_resolution=601,heatmap=False,sep_contour_levels_mps=[100.,0.,-100.,-200.,-400.],sweep_percent=0.,flaps_percent=0.,instructor=True,
                aircraft_settings={},compare_instructor=False,entries=None,instructor_model='steady',roll_leveling=False)

# Axes and sampling belong to the comparison; all physical conditions belong
# to an aircraft. Legacy configurations without overrides still apply to all.
AIRCRAFT_SETTINGS = frozenset(('altitude_m','fuel_percent','throttle','afterburner',
    'trim_mode','trim_limit','fixed_trim','extra_mass_kg','structural_limits',
    'timestep_hz','sweep_percent','flaps_percent','instructor','instructor_model','engine_control_mode','torque_gyro','roll_leveling'))


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
    """Validate all untrusted UI/CLI settings before any numerical work."""
    if values and values.get('entries') is not None:
        from em_entries import entry_settings
        return entry_settings(values)
    result=dict(DEFAULTS); result.update(values or {})
    result['structural_limits']=True  # Physical limits cannot be disabled.
    unknown=set(result)-set(DEFAULTS)
    if unknown: raise ValueError('Unknown setting: '+', '.join(sorted(unknown)))
    if not isinstance(result['aircraft'], list) or not result['aircraft'] or len(result['aircraft'])>2:
        raise ValueError('Select one or two aircraft')
    if any(a not in AIRCRAFT for a in result['aircraft']) or len(set(result['aircraft']))!=len(result['aircraft']):
        raise ValueError('Unknown or repeated aircraft')
    for name in result['aircraft']:
        if not AIRCRAFT[name]['supported']:raise ValueError(AIRCRAFT[name]['name']+': '+AIRCRAFT[name]['reason'])
    limits={'altitude_m': (0,18000), 'fuel_percent': (1,100), 'throttle': (0,1.1),
            'trim_limit': (0,1), 'extra_mass_kg': (0,5000), 'speed_min_kmh': (100,2500),
            'speed_max_kmh': (200,2600), 'max_load_g': (1.1,20),
            'speed_samples': (7,129), 'load_samples': (5,49), 'timestep_hz': (30,120),
            'sep_tolerance_mps':(.05,5.),'surface_resolution':(201,1201),'sweep_percent':(0,100),'flaps_percent':(0,100)}
    for key,(low,high) in limits.items():
        value=result[key]
        if key=='max_load_g' and value is None:continue
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not low<=value<=high:
            raise ValueError(f'{key} must be between {low} and {high}')
        result[key]=float(value)
    for key in ['speed_samples','load_samples','surface_resolution']:
        if int(result[key])!=result[key]: raise ValueError(key+' must be an integer')
        result[key]=int(result[key])
    result['sep_contour_levels_mps']=contour_levels(result['sep_contour_levels_mps'])
    if result['speed_min_kmh']>=result['speed_max_kmh']: raise ValueError('Maximum speed must exceed minimum speed')
    for key in ['afterburner','structural_limits','instructor','compare_instructor','torque_gyro','heatmap','roll_leveling']:
        if not isinstance(result[key],bool): raise ValueError(key+' must be true or false')
    if result['compare_instructor'] and len(result['aircraft'])!=1:
        raise ValueError('Select one aircraft to compare Instructor on/off')
    if result['engine_control_mode'] not in ('automatic','optimized'):raise ValueError('Unknown engine control mode')
    result['engine_control_mode']='automatic'
    if result['instructor_model']!='steady':raise ValueError('Only the static Instructor boundary is supported')
    if result['trim_mode'] not in ('optimized','fixed'): raise ValueError('Unknown trim mode')
    if result['sampling'] not in ('adaptive','regular'):raise ValueError('Unknown sampling mode')
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
        if (condition['instructor'] or result['compare_instructor']) and condition['extra_mass_kg']:
            raise ValueError('Instructor integration currently requires a clean loadout (zero extra mass)')
    return result


def turn_geometry(alpha_deg, bank_deg, speed, load, dt, ias_u, sideslip_deg=0.):
    """Invert the native Y-Z-X attitude increment for a constant-bank turn.

    Heading advances toward world +z, with physical angular axis world -y.
    Native damping acts before integration; balanced midpoint rate therefore
    differs from the stored rate. Float32 trig normalization remains in native
    kinematics; this analytic inverse is checked by its returned step residual.
    """
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
    # Target right-product quaternion has axis -up and angle turn_rate*dt.
    x,y,z=-up*math.sin(turn_rate*dt*.5); w=math.cos(turn_rate*dt*.5)
    r00=1-2*(y*y+z*z); r10=2*(x*y+w*z); r20=2*(x*z-w*y)
    r11=1-2*(x*x+z*z); r12=2*(y*z-w*x)
    yaw=math.atan2(-r20,r00); pitch=math.asin(max(-1.,min(1.,r10))); roll=math.atan2(-r12,r11)
    midpoint=-np.array([roll,yaw,pitch])/dt
    damping=max(1.-float(f32(ias_u))*1e-5,.8)
    stored=midpoint*(2./(1.+damping))
    # R = Rx(bank) Rz(alpha), at initial world heading zero.
    quaternion=[f32(math.sin(bank*.5)*math.cos(alpha*.5)),
                f32(-math.sin(bank*.5)*math.sin(alpha*.5)),
                f32(math.cos(bank*.5)*math.sin(alpha*.5)),
                f32(math.cos(bank*.5)*math.cos(alpha*.5))]
    if sideslip_deg:
        # R = Rx(bank) Ry(beta) Rz(alpha). Keep the original beta=0
        # arithmetic above exactly, including its float32 rounding.
        sx,cx=math.sin(bank*.5),math.cos(bank*.5)
        sy,cy=math.sin(beta*.5),math.cos(beta*.5)
        sz,cz=math.sin(alpha*.5),math.cos(alpha*.5)
        quaternion=list(map(f32,[sx*cy*cz+cx*sy*sz,cx*sy*cz-sx*cy*sz,
                                cx*cy*sz+sx*sy*cz,cx*cy*cz-sx*sy*sz]))
    return dict(forward=forward, normal=normal, side=side, up=up, lateral=lateral,
                omega=stored, quaternion=quaternion, turn_rate=turn_rate)


class EngineUnit:
    """Settled RPM cycle or deterministic time sample, then exact nozzle means.

    With ample fuel the RPM dynamics do not depend on table thrust. We retain
    old-RPM mode multipliers and apply each to the actual local table/nozzle.
    Each phase is rounded separately before taking the double ensemble mean.
    """
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
        # RPM settling does not read nozzle force/moment. Use a zero-ratio
        # diagnostic nozzle there; the vectors below use every real nozzle.
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
        # The table's fuel-limited target is the only speed-dependent input to
        # intact jet RPM dynamics (owner resets torque/friction every step).
        # Re-settle when it changes; never reuse a cycle from another target.
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
    """Individual jet/nozzle sums, followed by the owner's double aggregation."""
    def __init__(self,model,mass,config):
        self.units=[EngineUnit(model,mass,config,e) for _,e in installed_engines(model['fm']) if e['Main']['Type']=='Jet']
        if not self.units:raise ValueError('No installed jet engines')
        self.summary=dict(self.units[0].summary)
        self.summary.update(engine_count=len(self.units),engines=[u.summary for u in self.units],
                            nozzle_policy='Normal forward flight; VTOL, reverse and thrust-vectoring commands zero')
        if len(self.units)>1:self.summary['policy']=str(len(self.units))+' engines · '+', '.join(dict.fromkeys(u.summary['policy'] for u in self.units))

    def __getattr__(self,name):return getattr(self.units[0],name)

    @lru_cache(maxsize=2048)
    def vectors(self,body_u,flaps=0.):
        force=[0.]*3;moment=[0.]*3
        for unit in self.units:
            f,m=unit.vectors(body_u,flaps)
            for i in range(3):force[i]+=f[i];moment[i]+=m[i]
        return force,moment


def command_allocation(properties, coordinate, ranges, config):
    """Check physical command reachability; allocation never changes the FM input.

    The optimizer coordinates are delivered aerodynamic commands. Available
    stick/trim authority determines feasibility separately from equilibrium.
    Inverse allocation is retained only for independent native-control checks.
    """
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
    # Search the forward-flight attitude chart. These are numerical coordinate
    # guards, not stall limits; feasibility uses the native wing angles below.
    # A -6 degree floor clipped ordinary high-speed/full-flap equilibria.
    trim_bounds=((-89.,-15.,-1.,-1.,-1.),(89.,89.7,1.,1.,1.))

    def __init__(self, name, config):
        self.name=name; self.config=aircraft_settings(settings(config),name)
        # Apply this calculation's control-helper choice to a private FM copy.
        # Catalog aircraft are cached and must not change between jobs.
        self.fm=dict(load_aircraft(name),RollLeveling=self.config['roll_leveling'])
        self.flaps=f32(self.config['flaps_percent']/100.)
        self.instructor_profile=instructor_profile(self.fm)
        self.model=at_sweep(prepare(self.fm),self.config['sweep_percent']/100.); self.controls=control_properties(self.fm)
        self.sweep_rows=sweep_schedule(self.fm);self.has_sweep=len(self.model['wing_family'])>1
        self.is_prop=is_prop(name)
        # Snapshot producers 101a6a068..a0b1 and 101a6e918..e961 publish
        # fully deployed gear when the aircraft has no retraction control.
        self.gear=1. if self.is_prop and not self.fm['AvailableControls'].get('hasGearControl',True) else 0.
        if self.is_prop:self.mass=prop_mass_state(name,self.config['fuel_percent'],self.config['extra_mass_kg'])
        else:
            fuel=[f32(capacity*self.config['fuel_percent']/100.) for capacity in fuel_capacities(self.fm)]
            payloads=[dict(mass=self.config['extra_mass_kg'],position=self.fm['Mass']['CenterOfGravity'])] if self.config['extra_mass_kg'] else []
            self.mass=mass_evaluate(aircraft_properties(self.fm),fuel,payloads=payloads)
            self.mass['fuel_by_system']=fuel
        self.weight=self.mass['mass']*float(G); self.dt=f32(1/self.config['timestep_hz'])
        self.engine=PropellerEnsemble(name,self.model,self.mass,self.config) if self.is_prop else EngineEnsemble(self.model,self.mass,self.config)
        self.instructor_boundaries={}
        self.instructor_trim_entries={}
        self.sideslip_attitude_deg=0.
        self._sideslip_solvers={}

    def at_sideslip(self,angle):
        """Immutable flight-condition view with its own Instructor cache.

        The angle is the attitude parameter; air.beta is the native measured
        sideslip. Keep both in each solved point so replay is unambiguous.
        """
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
        """Replay a complete exported state, including its solved sideslip."""
        view=self.at_sideslip(point.get('sideslip_attitude_deg',0.))
        if self.is_prop and point.get('propulsion'):
            view=view.with_prop_controls(point['propulsion']['controls'])
        return view.operating_point(point['speed_kmh']/3.6,point['load_g'],point['solution'],canonical_propulsion=self.is_prop)

    def with_prop_controls(self,controls):
        import copy
        view=copy.copy(self);view.engine=self.engine.with_controls(controls)
        view.__dict__.pop('_trim_predictor',None)
        view.__dict__.pop('_physical_sideslip_recovery',None)
        view.instructor_boundaries={};view._sideslip_solvers={}
        # A canonical numerical replay with unchanged engine commands does
        # not change the Instructor's reference flight conditions. Preserve
        # that entry path across solver views; rebuilding it at every phase
        # correction repeated scores of identical level equilibria. Different
        # commands still get an independent entry path, as do sideslip views.
        same_controls=controls==(self.engine.fixed_controls or
            (self.engine.automatic_controls if self.engine.automatic else None))
        view.instructor_trim_entries=self.instructor_trim_entries if same_controls else {}
        view.__dict__.pop('_minimum_instructor_speed',None)
        return view

    def instructor_at(self,value,speed,load):
        from instructor_envelope import controller_limits
        return controller_limits(self,value,speed)

    def derivative_branch(self,value):
        """Identify native piecewise branches without changing their values."""
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
        """Solve equilibrium together with an active envelope-limit equation.

        This treats load as an unknown. A control stop, stall margin, or wing
        force limit closes the sixth equation; a failed optimizer is never an
        active constraint. Return a directly verified interior boundary point.
        """
        angle=low.get('sideslip_attitude_deg',self.sideslip_attitude_deg)
        if angle!=self.sideslip_attitude_deg:
            view=self.at_sideslip(angle)
            other=view.solve(speed_kmh,high['load_g'],high['solution'],exhaustive=False)
            return view.boundary(speed_kmh,low,other,continuation=continuation,scalar_fallback=scalar_fallback,
                                 candidate_kinds=candidate_kinds,max_iterations=max_iterations,
                                 trial_cycle_seconds=trial_cycle_seconds)
        if self.is_prop and not self.engine.automatic and self.engine.fixed_controls is None:
            from prop_trim import boundary as prop_boundary
            return prop_boundary(self,speed_kmh,low,high)
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
            # Use the regular turn coordinate before attempting the singular
            # load-coordinate search, rather than after all of its retries.
            from em_level_limit import near_level_boundary
            point=near_level_boundary(self,speed_kmh,low,high,candidate_kinds,max_iterations)
            if point:return point
        strength={'CritOverload':self.model['geometry']['strength']['force']}
        candidates=[]
        control_seed=high if high.get('converged') else low
        for i in range(2,5):
            lo,hi=high.get('control_bounds',[[-1.,1.]]*3)[i-2]
            # A synthetic or unbalanced high endpoint can still carry the
            # level-flight command sign, which points the active-control solve
            # at the opposite stop. The balanced interior point supplies the
            # observed direction of continuation.
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
        # A balanced rejected endpoint identifies an actual crossed constraint.
        # Try that equation before distant, still-inactive constraints.
        crossed=PHYSICAL_KINDS
        crossed_kind=crossed.get(high['reasons'][0]) if high['converged'] and len(high['reasons'])==1 else None
        # An outline probe between two verified limits of the same kind can
        # start on that constraint. The returned point must still close all
        # equations and keep every competing constraint safely inside bounds.
        crossed_kind=crossed_kind or high.get('continuation_limit_kind')
        if crossed_kind:candidates.sort(key=lambda candidate:candidate[0]!=crossed_kind)
        if candidate_kinds is not None:
            candidates=[candidate for candidate in candidates if candidate[0] in candidate_kinds]
        def isolated_limit(point,kind):
            # Only short-circuit inside the existing valid/rejected bracket,
            # with every other constraint safely inside its numerical inset.
            # Co-located limits and failed endpoints keep the full competition.
            return (low['load_g']<=point['load_g']<=high['load_g']
                and (kind=='stall' or point['stall_margin_deg']>.004)
                and (kind=='control' or point['authority_margin']>6e-5)
                and (kind=='pitch response' or (point.get('pitch_response') or {}).get('margin',0.)>4e-6)
                and (kind=='wing force' or not self.config['structural_limits'] or max(point['wing_load_ratios'])<.99990)
                and (kind=='Instructor pitch' or not self.config['instructor'] or point['instructor']['envelope_margin']>.0001))
        lower=np.array([*self.trim_bounds[0],1.]);upper=np.array([*self.trim_bounds[1],self.config['max_load_g'] or 64.])
        answers=[]
        self.boundary_diagnostics=[]
        # Near maximum lift, a small force residual can move alpha enough to
        # obscure the active constraint when propulsion settings are compared.
        # Propeller boundaries need the same tighter numerical target as their
        # subsequent control comparison; published acceptance stays unchanged.
        compare_prop_controls=self.is_prop and not self.engine.automatic
        boundary_force_goal=1e-6 if compare_prop_controls else 2e-4
        boundary_rate_goal=2e-6 if compare_prop_controls else 5e-5
        pitch_inset=min(2e-6,max(0.,low['instructor']['envelope_margin']*.5)) if self.config['instructor'] and low.get('instructor') else 2e-6
        response_inset=min(2e-6,(low.get('pitch_response') or {}).get('margin',4e-6)*.5)
        attempts=[(kind,axis,target,seed) for seed in [high,low] for kind,axis,target in candidates]
        # A balanced automatic-prop structural bracket already contains the
        # five-dimensional aircraft equilibria. Refining its remaining scalar
        # wing-load equation is both more robust and much cheaper than solving
        # a noisy six-dimensional governor/aircraft system first.
        scalar_wing_bracket=(self.is_prop and self.engine.automatic and crossed_kind=='wing force'
            and low['valid'] and high['converged'] and set(high['reasons'])<= {'wing force limit'}
            and max(low['wing_load_ratios'])<=.99995<max(high['wing_load_ratios']))
        if scalar_wing_bracket:attempts=[]
        if self.config['instructor']:
            # Try both local seeds before a different constraint can select a
            # reopened controller branch. A small interior inset also avoids
            # demanding a root inside the predictor's float32 command steps.
            attempts=[(kind,axis,inset,seed) for kind,axis,target in candidates
                      for inset in ([target,5e-5] if kind=='Instructor pitch' else [target])
                      for seed in [high,low]]
        # A later normal-response pocket can lie beyond a reversed-response
        # interval. Approach this physical limit from the certified interior,
        # before considering the extrapolated or disconnected outer seed.
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
            # Extrapolated continuation seeds may lie past a command stop.
            # Start the active-control solve on its known command coordinate;
            # differentiating an out-of-bounds saturated seed can otherwise
            # give a zero control derivative and stall at a valid endpoint.
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
                # Close the active equation on a reproducible complete native
                # phase mean. A warm output mean can move the final stall or
                # control margin after a seemingly successful boundary solve.
                # Switch only near an automatic-governor mean root; every
                # force, moment and constraint tolerance remains unchanged.
                if (equilibrium_solver is self and self.is_prop and self.engine.automatic
                        and not self.engine.force_canonical
                        and (value['propulsion'].get('stationarity') or {}).get('method')=='bounded native limit-cycle mean'
                        and value['force_error_g']<.001 and max(abs(value['rate_residual']))<.001
                        and abs(res[5])<.001):
                    equilibrium_solver=self.with_prop_controls(value['propulsion']['controls'])
                    equilibrium_solver.engine.force_canonical=True
                    memo.clear();res,value=evaluate_at(z)
                # Turn rate has a square-root slope at 1 g. Retain the tighter
                # target there even without a manual-control comparison.
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
                # Rank trial steps with one settled propulsion state, then
                # check candidates against the complete aircraft in that
                # order. All acceptance and published residuals remain fully
                # coupled; poor proxy rankings still try every original step.
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
            # A balanced rejected endpoint bounds the connected branch. Do not
            # accept a different authority/rounding root above that endpoint.
            if high['converged'] and high['reasons'] and z[5]>high['load_g']+1e-8:continue
            interior=equilibrium_solver.solve(speed_kmh,z[5],z[:5].tolist(),exhaustive=False,refine=compare_prop_controls or z[5]<1.05)
            if interior['valid']:
                # Final trim correction must preserve the active constraint
                # for every propulsion type, including nearly flat jet lift.
                if kind=='stall':checked_margin=(interior['stall_margin_deg']-.002)/10.
                elif kind=='wing force':checked_margin=max(interior['wing_load_ratios'])-.99995
                elif kind=='control':
                    bound=interior['control_bounds'][axis-2][1 if target>0 else 0]
                    checked_margin=interior['solution'][axis]-(bound-target*3e-5)
                elif kind=='pitch response':checked_margin=interior['pitch_response']['margin']-response_inset
                else:checked_margin=interior['instructor']['envelope_margin']-active_pitch_inset
                if abs(checked_margin)>=constraint_tolerance:continue
                interior['envelope_limit']=dict(kind=kind,axis=axis if kind=='control' else None,
                                                limiting_load_g=float(z[5]),constraint_residual=float(checked_margin),evaluations=len(memo))
                if kind=='Instructor pitch':
                    interior['envelope_limit']['pitch_inset']=active_pitch_inset
                if kind==crossed_kind and isolated_limit(interior,kind):return interior
                answers.append(interior)
        if not answers and self.config['structural_limits']:
            # The six-dimensional search can hit a native rounding branch
            # right at a wing-force limit. A scalar constraint solve along
            # independently balanced equilibria supplies a checked fallback.
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
                    # This is the same acceptance threshold used after brentq.
                    # Stop as soon as an independently balanced sample meets
                    # it instead of spending ~20 solves resolving float noise
                    # several orders below the plotted surface accuracy.
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
                    # Preserve a physical bracket even when a float32 branch
                    # prevents a continuous scalar root. Both endpoints must
                    # balance all forces/moments; the upper one must actually
                    # violate wing strength. A failed solve cannot be that end.
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
            # A distant rejected load can lie on a different tail-polar branch.
            # Follow balanced equilibria in alpha, with load an unknown, to get
            # a local seed for the SAME six-equation active-constraint solve.
            # This does not turn a last valid continuation point into a limit.
            recovered=self.continue_boundary(speed_kmh,low,high)
            if recovered:answers.append(recovered)
        return min(answers,key=lambda p:p['load_g']) if answers else None

    def refine_permission_boundary(self,speed_kmh,low,high,strict=False):
        """Refine a physical crossing without chasing rounded command zeros.

        Native controller outputs are piecewise float32 values. Their sign
        crossing may have no zero, so repeated high-accuracy inner solves and
        a near-machine-precision Brent root are neither necessary nor reliable.
        The shared bracket keeps fully balanced valid/rejected endpoints and
        stops on the existing load AND turn-rate boundary tolerances.
        """
        from em_constraint_bracket import refine
        return refine(self,speed_kmh,low,high)

    def continue_boundary(self,speed_kmh,low,high):
        point=low;step=.5
        lower=np.array([-15.,-1.,-1.,-1.,1.])
        upper=np.array([89.7,1.,1.,1.,self.config['max_load_g'] or 64.])
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
            # Reduce the continuation step near a control stop or sharp polar
            # transition. Failures remain numerical if no constraint closes.
            step*=.5
            if step<.0078125:
                limit=self.boundary(speed_kmh,point,high,continuation=False)
                if limit:
                    limit['envelope_limit']['seed_method']='balanced angle continuation'
                    limit['envelope_limit']['continuation_steps']=iteration+1
                return limit
        return None

    from em_operating import operating_point

    def initial_guess(self,speed,load):
        """Aircraft-specific seed below positive stall; not a lift-only solution.

        The old universal 25-degree seed already exceeded some aircraft's
        critical wing angle. Use the current wing polar and dynamic pressure
        to start below the positive critical angle. The negative critical
        angle is not a chart constraint or a lower bound on the search seed.
        """
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

    def solve(self, speed_kmh, load, initial=None, detailed=False, exhaustive=True, refine=False,quick=False):
        if self.is_prop and not self.engine.automatic and self.engine.fixed_controls is None:
            from prop_trim import solve as solve_prop_trim
            return solve_prop_trim(self,speed_kmh,load,initial,detailed,exhaustive,refine)
        seeded=initial is not None
        speed=speed_kmh/3.6; initial=initial or self.initial_guess(speed,load)
        memo={}; calls=0;last_jacobian=None
        def evaluate_x(x):
            nonlocal calls
            check_cancel()
            key=tuple(x)
            if key not in memo:
                if len(memo)>256:memo.clear()
                # Bounded native propagation supplies search residuals; the
                # final complete phase average independently checks balance.
                budget=(20. if quick or self.engine.prefer_short_search else 60.) if self.is_prop and self.engine.automatic else 60.
                memo[key]=self.operating_point(speed,load,x,cycle_seconds=budget); calls+=1
            return memo[key]
        def fun(x):return evaluate_x(x)['residual']
        def jac(x,freeze_propulsion=False):
            nonlocal last_jacobian
            base=evaluate_x(x); steps=[.002,.002,.0002,.0002,.0002];columns=[]
            derivative_base=(self.operating_point(speed,load,x,propulsion_override=base['propulsion'])
                             if freeze_propulsion else base)
            # Differentiate on the same rounded-Mach branch. Changing attitude
            # can toggle Mach by an ulp at fixed world TAS; the exact native Cm
            # polynomial is occasionally too discontinuous for a naive finite
            # difference. Never smooth/replace the function being solved.
            for i,h in enumerate(steps):
                for factor in [1.,.5,1.5,2.,.25,-.5,-1.,-1.5,-2.,.1,3.,4.,5.,-.1,.05,-.05,.01,-.01]:
                    step=h*factor;trial=np.asarray(x)+np.eye(5)[i]*step
                    sample=(self.operating_point(speed,load,trial,propulsion_override=base['propulsion'])
                            if freeze_propulsion else evaluate_x(trial))
                    if self.derivative_branch(sample)==self.derivative_branch(derivative_base):break
                columns.append((sample['residual']-derivative_base['residual'])/step)
            last_jacobian=np.column_stack(columns)
            return last_jacobian
        bounds=self.trim_bounds
        x0=np.clip(initial,np.asarray(bounds[0])+1e-6,np.asarray(bounds[1])-1e-6)
        bound_stationary=False
        def direction(matrix,residual,x,scale):
            """Solve the bounded linear correction, including active controls.

            Clipping an unconstrained Newton step drops its control correction
            without rebalancing the other unknowns. Repeated full propulsion
            Jacobians then try to push through the same actuator stop. BVLS
            re-solves the remaining directions on the active constraint.
            """
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
            # A steep outer controller equation needs more accurate inner
            # aircraft roots than a plotted interior sample. This changes
            # numerical stopping only; final physical acceptance below keeps
            # the original force, moment and history tolerances.
            return (value['force_error_g']<=(1e-6 if refine else 2e-4) and
                    max(abs(value['rate_residual']))<=(2e-6 if refine else 5e-5) and
                    value['force_error_g']+value['force_mean_uncertainty_g']<=2e-4 and
                    max(abs(value['rate_residual'])+value['angular_mean_uncertainty_rad_s2'])<=5e-5)
        class ClosedEquilibrium(Exception):
            def __init__(self,x,value):self.x=np.array(x);self.value=value
        def fit_equilibrium(_fun,x,**options):
            # The trust-region optimizer otherwise pursues its generic cost
            # tolerance far below the native rounding floor even after the
            # full aircraft has closed. Stop on the existing physical targets.
            def checked(q):
                v=evaluate_x(q)
                if balanced(v) and v['history_error']<=2e-4:
                    raise ClosedEquilibrium(q,v)
                return v['residual']
            try:return least_squares(checked,x,**options)
            except ClosedEquilibrium as closed:
                return SimpleNamespace(x=closed.x,fun=closed.value['residual'])
        # Nearby continuation points usually need only two Newton corrections.
        # Stop at the physical closure tolerances, rather than asking a generic
        # optimizer to resolve noise beneath the native float32 rounding floor.
        x=x0.copy(); value=evaluate_x(x)
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
                    q=np.clip(x+factor*delta,*bounds);v=evaluate_x(q)
                    if balanced(v) or np.linalg.norm(v['residual'])<np.linalg.norm(value['residual']):
                        dx=q-x;den=float(dx.dot(dx))
                        if den>1e-12:
                            matrix+=np.outer(v['residual']-value['residual']-matrix.dot(dx),dx)/den
                        x,value=q,v;advanced=True;break
                if not advanced:break
            if balanced(value):last_jacobian=matrix
        for iteration in range(0 if prop_blocked else 8):
            if balanced(value):break
            # An inexact Newton direction need not re-settle the engine for
            # every finite difference. A frozen mean supplies only this local
            # search direction; every trial/accepted residual below still
            # re-solves propulsion and evaluates the full nonlinear cycle.
            # The existing fully coupled Jacobian remains the fallback.
            fast_jacobian=self.is_prop and self.engine.automatic
            if fast_jacobian:
                # Correct a cheap frozen-propulsion model to the exact phase
                # residual at this iterate. Several inexpensive inner Newton
                # steps give a better direction without repeatedly settling
                # the governor. This is a predictor only: the line search and
                # final canonical replay below use the complete equations.
                frozen=value['propulsion'];inner=x.copy()
                proxy=self.operating_point(speed,load,inner,propulsion_override=frozen)
                offset=value['residual']-proxy['residual'];res=proxy['residual']+offset
                for _ in range(5):
                    columns=[]
                    for i,h in enumerate([.002,.002,.0002,.0002,.0002]):
                        for factor in [1.,.5,2.,-.5,-1.,.1,-.1,.01,-.01]:
                            step=h*factor;trial=inner.copy();trial[i]+=step
                            vv=self.operating_point(speed,load,trial,propulsion_override=frozen)
                            if self.derivative_branch(vv)==self.derivative_branch(proxy):break
                        columns.append((vv['residual']-proxy['residual'])/step)
                    last_jacobian=np.column_stack(columns)
                    correction=direction(last_jacobian,res,inner,[8.,20.,.4,.4,.4])
                    advanced=False
                    for factor in [1.,.5,.25,.1]:
                        trial=np.clip(inner+factor*correction,*bounds)
                        vv=self.operating_point(speed,load,trial,propulsion_override=frozen)
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
                trial_x=np.clip(x+delta*factor,*bounds);trial=evaluate_x(trial_x)
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
            point=strict.solve(speed_kmh,load,x.tolist(),detailed=detailed,
                               exhaustive=False,refine=refine)
            if point['valid']:
                point['evaluations']+=calls
                return point
        if not quick and not prop_blocked and not bound_stationary and not balanced(value) and self.is_prop and self.engine.automatic:
            # A bounded inexact-Jacobian attempt avoids re-settling propulsion
            # for every derivative in the trust-region search. Every residual
            # is fully coupled. Failure retains the original coupled fallback.
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
            # A balanced post-stall root can coexist with the requested
            # pre-stall branch. Restart below stall even when that rejected
            # root has a smaller residual; residual norm is not branch choice.
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
            # A trust-region step can stop at a rounded-Mach discontinuity.
            # Polish nearby states with the same native residual/Jacobian and
            # an explicit line search. Tiny bank restarts sample neighboring
            # rounding branches; every accepted point still meets all checks.
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
            # Certify the continued native engine branch using its complete
            # phase outputs, including the nonlinear aircraft consumer.
            canonical=(self.operating_point(speed,load,result.x,cycle_seconds=20.,phase_certificate=True)
                       if self.engine.automatic else
                       self.operating_point(speed,load,result.x,canonical_propulsion=True))
            if self.engine.automatic and not canonical['propulsion']['converged']:
                # Complete the same native transient before trying another
                # initialization. Restarting a slowly damping governor here
                # can reproduce the same timeout forever at valid low loads.
                canonical=self.operating_point(speed,load,result.x,cycle_seconds=180.,phase_certificate=True)
            if self.engine.automatic and not canonical['propulsion']['converged']:
                canonical=self.operating_point(speed,load,result.x,canonical_propulsion=True)
            # Most propeller states close an exact native cycle. Only a
            # balanced state that fails that replay needs the bounded native
            # output certificate; keep the usual path and its cache intact.
            if not self.engine.automatic and balanced(final) and not canonical['propulsion']['converged']:
                certified=self.operating_point(speed,load,result.x,canonical_propulsion=True,
                    certify_stationary=True)
                if certified['propulsion']['converged']:canonical=certified
            # The final phase average can differ from the mean used during
            # search. Correct the aircraft against that complete phase average
            # with all original force/moment tolerances unchanged.
            near_closure=(canonical['propulsion']['converged'] and
                          canonical['force_error_g']<.015 and max(abs(canonical['rate_residual']))<.008)
            if not bound_stationary and not balanced(canonical) and (balanced(final) or near_closure):
                strict=self.with_prop_controls(canonical['propulsion']['controls'])
                # A changing warm-cycle averaging phase makes this last small
                # correction noisy. Use one deterministic, complete native
                # replay solve rather than repeated warm corrections followed
                # by alpha/sideslip restarts. Acceptance tolerances are intact.
                strict.engine.force_canonical=True
                point=strict.solve(speed_kmh,load,result.x.tolist(),detailed=detailed,exhaustive=exhaustive,refine=refine,quick=quick)
                point['evaluations']+=calls;return point
            final=canonical
        aero=final['result']; geom=final['geometry']; strength=self.model['geometry']['strength']
        ratios=final['maximum_wing_load_ratios']
        converged=(final['force_error_g']+final['force_mean_uncertainty_g']<=2e-4 and
                   max(abs(final['rate_residual'])+final['angular_mean_uncertainty_rad_s2'])<=5e-5 and final['history_error']<=2e-4)
        reasons=[]
        if not converged:reasons.append('trim did not converge')
        if self.is_prop:
            propulsion=final['propulsion']
            if not propulsion['converged']:reasons.append('propulsion did not settle')
            # RPMMaxAllowed is not an instantaneous native flight limit.
            # Intact-engine EM fixes health; retain RPM diagnostics below.
            if propulsion['stopped_shafts']:reasons.append('propeller shaft stopped')
            phases=propulsion.get('cycle_samples')
            phase_check=dict(checked=False,force_error_g=None,angular_error_rad_s2=None,history_error=None)
            stationary_mean=propulsion.get('stationarity',{}).get('method')=='bounded native limit-cycle mean' if propulsion.get('stationarity') else False
            if not phases and not stationary_mean:reasons.append('propulsion cycle unresolved')
            elif converged:
                phase_check=dict(checked=True,model=('Bounded native propulsion time mean at the prescribed flight condition'
                    if stationary_mean else 'Mean of complete aircraft phase outputs at the prescribed flight condition'),
                    force_error_g=final['force_error_g'],angular_error_rad_s2=float(max(abs(final['rate_residual']))),
                    force_mean_uncertainty_g=final['force_mean_uncertainty_g'],
                    angular_mean_uncertainty_rad_s2=final['angular_mean_uncertainty_rad_s2'].tolist(),
                    history_error=final['history_error'],periodic_aircraft_trajectory_certified=False)
        if not self.config['instructor'] and not final['allocation']['reachable']:reasons.append('control authority')
        if final['stall_margin']<0:reasons.append('post-stall')
        if abs(final['kinematic']['velocity'][1])>.005:reasons.append('vertical step did not close')
        structural=[]
        if max(ratios)>1:structural.append('wing force limit')
        if aero['air']['ias_u']>strength['ias']:structural.append('IAS limit')
        if aero['air']['mach']>strength['mach']:structural.append('Mach limit')
        sweep_range=available_sweep(self.fm,self.sweep_rows,aero['air']['mach']) if self.has_sweep else (0.,1.)
        if self.has_sweep and not sweep_range[0]-1e-7<=self.model['sweep']<=sweep_range[1]+1e-7:reasons.append('sweep unavailable')
        if self.config['structural_limits']:reasons+=structural
        pitch=None
        if converged and final['stall_margin']>=0. and not reasons:
            pitch=pitch_response(self,final)
            if not pitch['normal']:
                if pitch['reversed']:
                    recovered=recover_pitch_response(self,final,detailed=detailed,refine=refine)
                    if recovered is not None:
                        recovered['evaluations']+=calls
                        return recovered
                reasons.append('reversed pitch response' if pitch['reversed'] else 'pitch response unresolved')
        instructor=None
        if self.config['instructor'] and not reasons:
            # First establish a physical operating point. A post-stall,
            # structurally excluded or unbalanced iterate must not turn an
            # entire speed column into an unresolved Instructor boundary.
            instructor=self.instructor_at(final,speed,load)
            if not instructor['converged']:reasons.append('Instructor boundary unresolved')
            else:
                if instructor['envelope_margin']<-2e-7:reasons.append('Instructor pitch limit')
                if instructor['margins']['control authority with auto trim']<-2e-7:reasons.append('control authority')
        control_bounds=instructor['control_bounds'] if instructor is not None else final['allocation']['bounds']
        point=dict(speed_kmh=float(speed_kmh),load_g=float(load),turn_dps=math.degrees(geom['turn_rate']),
                   ps_mps=final['ps'],ps_continuous_mps=final['ps_continuous'],alpha_deg=float(result.x[0]),bank_deg=float(result.x[1]),
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
                   component_forces=aero['component_forces'],evaluations=calls,solution=result.x.tolist(),
                   vertical_step_velocity_mps=final['kinematic']['velocity'][1],
                   altitude_correction=final['altitude_correction'],
                   actual_turn_dps=math.degrees(math.atan2(final['kinematic']['velocity'][2],final['kinematic']['velocity'][0])/self.dt))
        if self.is_prop:
            p=final['propulsion']
            if converged and p['converged'] and phase_check['checked'] and self.engine.automatic:
                # Native continuation state is private numerical evidence.
                # A later nearby task can start here, then must satisfy the
                # same settling and complete-aircraft phase certificate.
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
            point['propulsion']['optimization']=dict(method='Native automatic engine controls',evaluated=0,
                global_optimum_certified=False,radiators_closed=True)
        if (self.config['instructor'] and not refine and not getattr(self,'_instructor_rounding_search',False) and point['converged']
                and point['reasons']==['Instructor pitch limit']
                and point['instructor'].get('pitch_predictor_recheck',True)
                and point['instructor']['pitch_margin']<-2e-7):
            # Native nested pitch predictors can switch to zero delivery for
            # the last few rounded bits of a barely closed aircraft state.
            # A large command deficit therefore does not prove an exclusion.
            # Polish the SAME equilibrium once, using the existing tighter
            # force/moment targets and full native controller/phase replay.
            # Bounded work; an unsuccessful polish retains the real rejection.
            corrected=self.solve(speed_kmh,load,point['solution'],detailed=True,
                                 exhaustive=False,refine=True,quick=True)
            if (not corrected['valid'] and point['instructor']['angle_margin']>.05
                    and point['authority_margin']>.01
                    and abs(point['instructor']['delivered_pitch']-point['instructor']['auto_trim'][1])<1e-7):
                # An exact float32 predictor branch can still differ between
                # equally balanced rounded states. Check neighboring native
                # representations, with every original feasibility and phase
                # certificate active. No control output is substituted.
                import copy
                original_corrected=corrected
                for origin,condition in ((point,final),(original_corrected,original_corrected['_detail'])):
                    nearby=copy.copy(self)
                    if self.is_prop:
                        nearby=self.with_prop_controls(origin['propulsion']['controls'])
                        nearby.engine._reference=copy.deepcopy(condition['propulsion']['state'])
                        nearby.engine.force_canonical=True
                    nearby._instructor_rounding_search=True
                    # Expand in native representable values, still far below
                    # plotted angular/control resolution. Larger searches are
                    # restricted to a collapsed protected-pitch prediction.
                    ctl=origin.get('instructor') or point['instructor']
                    collapsed=abs(ctl['delivered_pitch']-ctl['auto_trim'][1])<1e-7
                    for scale in ((8.,64.,512.) if collapsed else (8.,)):
                        for axis in (3,0,1):
                            step=scale*max(abs(float(np.spacing(np.float32(origin['solution'][axis])))),1e-10)
                            for sign in (-1.,1.):
                                guess=list(origin['solution']);guess[axis]+=sign*step
                                trial=nearby.solve(speed_kmh,load,guess,detailed=detailed,exhaustive=False,quick=True)
                                if trial['valid']:corrected=trial;break
                            if corrected['valid']:break
                        if corrected['valid']:break
                    if corrected['valid']:break
                if (not corrected['valid'] and self.is_prop and self.engine.automatic
                        and not getattr(self,'_instructor_independent_recheck',False)):
                    # A warm governor can select a different, equally settled
                    # float32 cycle. Near a discontinuity in the native pitch
                    # predictor, that tiny force difference changes delivery.
                    # Before declaring an exclusion, replay an independent
                    # drivetrain AND controller entry. This is a bounded root
                    # restart, not a substituted controller output; the same
                    # complete aircraft and native-history checks must pass.
                    independent=self.with_prop_controls(point['propulsion']['controls'])
                    independent.engine._warm=None;independent.engine._reference=None
                    independent.engine._task_seed=None;independent.engine.force_canonical=False
                    independent.engine.prefer_short_search=False;independent.engine.search_cycle_seconds=20.
                    # These histories already start from their own canonical
                    # reference path, independent of incoming engine state.
                    # Rebuilding that identical path for each rejected load
                    # is redundant and can dominate a partial-flap column.
                    independent.instructor_trim_entries=self.instructor_trim_entries
                    independent._instructor_independent_recheck=True
                    trial=independent.solve(speed_kmh,load,point['solution'],
                        detailed=detailed,exhaustive=False,quick=True)
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
            # This matrix is a predictor, never a physical acceptance test.
            # Every trial and canonical replay still uses the full equations;
            # unsuccessful predictors enter the unchanged derivative fallback.
            self._trim_predictor=((speed_kmh,self.sideslip_attitude_deg),last_jacobian,load)
        if detailed:point['_detail']=final
        if exhaustive and not prop_blocked and not bound_stationary and seeded and (not converged or 'post-stall' in reasons):
            # A supplied continuation state must not make recovery less robust
            # than an independent solve (which includes its own Newton start).
            independent=self.solve(speed_kmh,load,detailed=detailed,exhaustive=True)
            if independent['valid']:
                independent['evaluations']+=calls
                independent['recovery']='independent equilibrium restart'
                return independent
        if exhaustive and not prop_blocked and not bound_stationary and not converged and final['force_error_g']<.02 and max(abs(final['rate_residual']))<.01:
            # Leave a stalled least-squares basin near a discontinuous native
            # branch. Every retry uses unchanged equations and acceptance
            # thresholds; prefer physical closure over a smaller residual norm.
            for alpha_offset in [.08,-.08,.3,-.3]:
                guess=result.x.copy();guess[0]+=alpha_offset
                retry=self.solve(speed_kmh,load,guess.tolist(),detailed=detailed,exhaustive=False)
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
    output=dict(settings=config,speeds_kmh=speeds,loads_g=loads,aircraft=[],method='coordinated trim below positive stall; native-step Ps; mean settled engine',
                assumptions=['Full-real manual aerodynamic trim','Full pilot authority; aerodynamic control-power loss retained',
                             'Constant fuel and intact components','Positive-AoA stall enforced; negative-AoA stall not an exclusion; native aerodynamics unchanged','Still air; out of ground effect; retracted gear/brake',
                             'Requested flap percentage held at every operating point, assumed achievable; intact flaps, no travel time or damage',
                             'Steady Instructor AoA schedule approximation; transient overshoot, delay and control history omitted' if config['instructor'] else 'Instructor off',
                             'Extra mass is a point mass at configured CG; ammunition is not inferred'],
                validation='Reconstructed kernels have native-code comparisons; this EM solver has not been validated against live flight.')
    total=len(speeds)*len(loads)*len(config['aircraft']); done=0
    for index,name in enumerate(config['aircraft']):
        solver=TrimSolver(name,config); columns=[]
        for i,speed in enumerate(speeds):
            column=[]
            for j,load in enumerate(loads):
                if cancelled and cancelled():raise InterruptedError('Calculation cancelled')
                initial=None
                if j and column[-1]['converged']:initial=column[-1]['solution']
                elif i and columns[-1][j]['converged']:initial=columns[-1][j]['solution']
                point=solver.solve(speed,load,initial); column.append(point); done+=1
                if progress:progress(dict(done=done,total=total,aircraft=name,speed_kmh=speed,load_g=load,
                                          valid=point['valid'],elapsed_s=time.monotonic()-start))
            columns.append(column)
        sustained=[]
        for i,column in enumerate(columns):
            for low,high in zip(column,column[1:]):
                # A coarse load grid can step from positive Ps directly past
                # stall/authority/structural bounds. Search the still-valid
                # side of that interval before deciding no root is bracketed.
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
        output['aircraft'].append(dict(id=name,**metadata,settings=solver.config,mass=solver.mass,engine=solver.engine.summary,points=points,
                                       instructor_approximation=instructor_profile(solver.fm) if solver.config['instructor'] else None,
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
