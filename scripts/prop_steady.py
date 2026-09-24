"""Settled installed propulsion under prescribed full body flow and rotation.

This is the temporal consumer used to validate proposed operating points. It
never replaces a shaft equilibrium by prescribed rated RPM. Output means carry
convergence and RPM evidence; a failed settle is not valid performance.
Engine health is fixed intact: RPMMaxAllowed is diagnostic, not a trim cutoff.
Control optimization and aircraft trim remain separate callers.
"""
import copy,math
from component_assembly import f32,add,mul
from piston_model import inlet_pressure,mixture
from propulsion_general import step,target_omega


def mixture_setting(p,velocity,height):
    if p['mixer_type']!=2:return f32(.5)
    inlet=inlet_pressure(height,velocity[0],p['ram_recovery'])
    candidates=[]
    for index in range(1,256):
        value=mul(float(index),f32(.005));r=mixture(p,inlet,value)
        if not r['requires_stop']:candidates.append((r['multiplier'],-abs(index-100),value))
    if not candidates:raise ValueError('No healthy delivered mixture at this inlet pressure')
    return max(candidates)[2]


def initial_state(p,velocity,height,throttle=1.1,afterburner=True,commands=None,gears=None,automatic=None,nitro=0.,engine_control_mode="optimized"):
    if engine_control_mode not in ("automatic","optimized"):raise ValueError("Unknown engine control mode")
    aec=engine_control_mode=="automatic"
    engines=[]
    for i,e in enumerate(p['engines']):
        ep=e['properties'];t=f32(0. if e['family']==3 else throttle)
        engines.append(dict(throttle=t,effective_throttle=t,running=0 if e['family']==3 else 7,afterburner=bool(afterburner),
            omega=ep['max_omega'],mechanical=1.,mixture=1. if aec else mixture_setting(ep,velocity,height),
            automatic_mixture=aec,automatic_compressor=aec,
            reservoir=ep['reservoir_capacity'],gear=gears[i] if gears is not None else 0,
            regulator=-1.,turbo=ep['turbo_min'],automatic_turbo=True))
    props=[]
    for i,prop in enumerate(p['propellers']):
        pp=prop['properties'];auto=(automatic[i] if automatic is not None else (aec or prop['automatic']))
        if auto and not aec and not prop['automatic']:raise ValueError('Automatic propeller control is unavailable')
        command=commands[i] if commands is not None else 255
        if not isinstance(command,int) or not 0<=command<=255:raise ValueError('Propeller command must be a delivered byte')
        if not auto and not prop['manual'] and command!=255:raise ValueError('Manual propeller control is unavailable')
        pitch=pp['pitch_min']
        props.append(dict(command=mul(float(command),f32(1/255)),auto=bool(auto),
                          pitch=pitch,governor_pitch=pitch,flow=[0.,0.,0.]))
    transmissions=[]
    for t in p['transmissions']:
        desired=[target_omega(p['engines'][l['index']]['properties'],engines[l['index']],nitro)*l['inverse_ratio'] for l in t['engines']]
        omega=f32(max(desired))
        transmissions.append(dict(omega=omega,previous_omega=omega))
    return dict(engines=engines,propellers=props,transmissions=transmissions,seed=12345)


def run_frames(p,state,frames,velocity,height,body_omega,cg,dt,nitro,torque_gyro=True):
    """Unmodified frame equations, double accumulation of published outputs."""
    count=0;total=[0.]*11;lo=[math.inf]*11;hi=[-math.inf]*11
    engine_max=[0.]*len(p['engines']);shaft_min=[math.inf]*len(p['transmissions'])
    for _ in range(frames):
        state=step(p,state,velocity,height,body_omega,cg,dt,state.get('seed',12345),nitro,torque_gyro=torque_gyro)
        row=state['aggregate_force']+state['aggregate_moment']+state['engine_angular_momentum']+state['engine_wash']
        if not all(math.isfinite(x) for x in row):raise ValueError('Nonfinite propulsion state')
        for i,x in enumerate(row):total[i]+=x;lo[i]=min(lo[i],x);hi[i]=max(hi[i],x)
        for i,e in enumerate(state['engines']):engine_max[i]=max(engine_max[i],e['omega'])
        for i,t in enumerate(state['transmissions']):shaft_min[i]=min(shaft_min[i],t['omega'])
        count+=1
    return state,dict(mean=[x/count for x in total],minimum=lo,maximum=hi,
                      engine_max_omega=engine_max,shaft_min_omega=shaft_min,frames=count)


def retained_key(state):
    """All time-dependent inputs read by the healthy propulsion equations."""
    values=[state.get('seed',12345)]
    for t in state['transmissions']:values.extend([t['omega'],t.get('previous_omega',t['omega'])])
    for p in state['propellers']:values.extend([p['pitch'],p.get('governor_pitch',p['pitch']),*p['flow']])
    for e in state['engines']:
        values.extend(e.get(k,0.) for k in ['omega','effective_throttle','torque','friction','regulator','gear',
            'turbo','turbo_command','mechanical','extra_amplitude','reservoir'])
    return tuple(values)


def saturated_key(p,key,previous):
    """Equivalent retained state behind a continuously active blade stop.

    An outward-moving governor command beyond the delivered pitch limit cannot
    change blade forces. Keep every other retained input exact, including RNG,
    RPM and wash. Inward motion cannot use this equivalence: it may release the
    stop. This changes only cycle detection, never the propagated native state.
    """
    if previous is None:return None
    projected=None;saturated=[]
    for i,prop in enumerate(p['propellers']):
        pitch=1+2*len(p['transmissions'])+5*i;gp=pitch+1
        lo,hi=prop['properties']['pitch_min'],prop['properties']['pitch_max']
        if key[pitch]==lo and key[gp]<lo and key[gp]<=previous[gp]:
            if projected is None:projected=list(key)
            projected[gp]=lo;saturated.append(gp)
        elif key[pitch]==hi and key[gp]>hi and key[gp]>=previous[gp]:
            if projected is None:projected=list(key)
            projected[gp]=hi;saturated.append(gp)
    return (tuple(saturated),tuple(projected)) if saturated else None


def settled_cycle(p,velocity,height,body_omega=(0.,0.,0.),cg=(0.,0.,0.),dt=1/48,nitro=0.,controls=None,max_seconds=60.,state=None,require_cycle=False,allow_stationary=False,torque_gyro=True,allow_stationary_mean=False,cold_start=False,aircraft_residual_scales=None):
    """Average an exact native cycle or a converged native limit-cycle mean.

    A float32 governor can remain on a small, stable limit cycle without its
    complete retained state ever repeating bit for bit.  Requiring an exact
    recurrence in that case rejects a physical settled state and makes the EM
    sampler subdivide the resulting artificial holes.  The fallback below
    retains native frame propagation and accepts only after two consecutive
    window-to-window means and the oscillation envelope have stabilized.
    """
    dt=f32(dt);warm_start=state is not None and not cold_start;initial=initial_state(p,velocity,height,nitro=nitro,**(controls or {}))
    if state is None:state=initial
    else:
        state=copy.deepcopy(state)
        for e,s,c in zip(p['engines'],state['engines'],initial['engines']):
            for k in ['throttle','afterburner','mixture','automatic_turbo','automatic_mixture','automatic_compressor']:s[k]=c[k]
            if e['properties']['manual_compressor'] and not c['automatic_compressor']:s['gear']=c['gear']
        for s,c in zip(state['propellers'],initial['propellers']):
            s['command']=c['command'];s['auto']=c['auto']
    seen={};equivalent={};output_seen={};output_keys=[];previous=None;last_saturated=();outputs=[];engine_speeds=[];shaft_speeds=[]
    # A nearby settled state only needs short confirmation windows. Published
    # points retain the full native phase outputs for aircraft certification.
    # A continued state is already on (or close to) the neighboring native
    # attractor. Short windows cover many governor oscillations and retain the
    # same two-mean plus envelope checks; a cold start keeps the longer window.
    window=max(1,round((2. if warm_start else 10.)/dt))
    for index in range(max(1,round(max_seconds/dt))):
        key=retained_key(state)
        projected=saturated_key(p,key,previous) if allow_stationary else None
        # A reversal/release invalidates every earlier projected recurrence.
        if projected is None or projected[0]!=last_saturated:
            equivalent.clear()
        last_saturated=projected[0] if projected else ()
        exact=key in seen
        if exact or projected is not None and projected in equivalent:
            start=seen[key] if exact else equivalent[projected];rows=outputs[start:];period=len(rows)
            mean=[sum(r[i] for r in rows)/period for i in range(11)]
            overspeed=[i for i,e in enumerate(p['engines']) if max(r[i] for r in engine_speeds[start:])>e['properties']['omega_limit']*(1.+1e-6)]
            stopped=[i for i in range(len(p['transmissions'])) if min(r[i] for r in shaft_speeds[start:])<1.]
            return dict(state=state,force=mean[:3],moment=mean[3:6],angular_momentum=mean[6:9],wash=mean[9:],
                converged=True,feasible=not stopped,overspeed_engines=overspeed,stopped_shafts=stopped,
                period_frames=period if exact else None,cycle_samples=rows,
                stationarity=None if exact else dict(method='repeating outputs behind saturated governor stop',
                    output_period_frames=period,saturated_governor_coordinates=list(projected[0]),
                    retained_state_comparison='exact except outward governor windup behind active blade stop'),
                window_relative_change=0.,simulated_seconds=index*dt)
        seen[key]=index
        if projected is not None:equivalent[projected]=index
        previous=key
        state=step(p,state,velocity,height,body_omega,cg,dt,state.get('seed',12345),nitro,torque_gyro=torque_gyro)
        row=state['aggregate_force']+state['aggregate_moment']+state['engine_angular_momentum']+state['engine_wash']
        if not all(math.isfinite(x) for x in row):raise ValueError('Nonfinite propulsion state')
        outputs.append(row);engine_speeds.append([e['omega'] for e in state['engines']]);shaft_speeds.append([t['omega'] for t in state['transmissions']])
        output_key=tuple(row);output_keys.append(output_key)
        prior_output=output_seen.get(output_key);output_seen[output_key]=len(outputs)-1
        # Some governors carry a harmless slowly drifting internal coordinate
        # while their complete delivered force/wash sequence is already an
        # exact native cycle. Confirm two whole, nontrivial output cycles
        # before accepting that observable recurrence.
        if allow_stationary_mean and prior_output is not None:
            period=len(outputs)-1-prior_output
            minimum_age=4*window if warm_start else max(4*window,round(40./dt))
            if (period>=max(4,round(.25/dt)) and period<=round(12./dt) and len(outputs)>=minimum_age
                    and len(outputs)>=2*period and output_keys[-period:]==output_keys[-2*period:-period]):
                rows=outputs[-period:];mean=[sum(r[i] for r in rows)/period for i in range(11)]
                overspeed=[i for i,e in enumerate(p['engines']) if max(r[i] for r in engine_speeds[-period:])>e['properties']['omega_limit']*(1.+1e-6)]
                stopped=[i for i in range(len(p['transmissions'])) if min(r[i] for r in shaft_speeds[-period:])<1.]
                return dict(state=state,force=mean[:3],moment=mean[3:6],angular_momentum=mean[6:9],wash=mean[9:],
                    converged=True,feasible=not stopped,overspeed_engines=overspeed,stopped_shafts=stopped,
                    period_frames=None,cycle_samples=rows,stationarity=dict(method='repeating native output cycle',
                        output_period_frames=period,retained_state_comparison='not required; two complete delivered-output cycles matched exactly'),
                    window_relative_change=0.,simulated_seconds=(index+1)*dt)
        # Give native retained-state cycles their usual settling time before
        # accepting a time mean; a cycle that closes later carries its exact
        # phase outputs into the nonlinear aircraft consumer.
        if allow_stationary_mean and (index+1)%window==0 and index+1>=3*window:
            # Prefer the same certificate with windows ending at a recurring
            # native output phase. Fixed 2/10-second windows can alias a
            # perfectly settled governor cycle. This also retains fewer
            # complete phase frames for the nonlinear aircraft consumer.
            from prop_cycle import aligned_window
            aligned=aligned_window(outputs,window,max(4,round(.25/dt)))
            if aligned is not None:
                period=aligned['frames'];rows=outputs[-2*period:]
                mean=[sum(r[i] for r in rows)/len(rows) for i in range(11)]
                overspeed=[i for i,e in enumerate(p['engines']) if max(r[i] for r in engine_speeds[-len(rows):])>e['properties']['omega_limit']*(1.+1e-6)]
                stopped=[i for i in range(len(p['transmissions'])) if min(r[i] for r in shaft_speeds[-len(rows):])<1.]
                return dict(state=state,force=mean[:3],moment=mean[3:6],angular_momentum=mean[6:9],wash=mean[9:],
                    converged=True,feasible=not stopped,overspeed_engines=overspeed,stopped_shafts=stopped,
                    period_frames=None,cycle_samples=rows,stationarity=dict(method='bounded native limit-cycle mean',
                        window_seconds=len(rows)*dt,comparison_window_seconds=period*dt,
                        window_alignment='recurring native output phase',
                        waveform_relative_change=aligned['waveform'],waveform_relative_change_limit=5e-3,
                        window_relative_changes=aligned['changes'],output_range=aligned['spread'],
                        envelope_relative_change=aligned['envelope'],mean_relative_change_limit=7e-4,
                        envelope_relative_change_limit=1e-3),
                    window_relative_change=max(aligned['changes']),simulated_seconds=(index+1)*dt)
            oldest=outputs[-3*window:-2*window];earlier=outputs[-2*window:-window];recent=outputs[-window:]
            means=[[sum(r[i] for r in rows)/window for i in range(11)] for rows in (oldest,earlier,recent)]
            before=means[-2]
            scales=[max(100.,*(abs(m[i]) for m in means)) if i<9 else max(1.,*(abs(m[i]) for m in means)) for i in range(11)]
            changes=[max(abs(means[j][i]-means[j-1][i])/scales[i] for i in range(11)) for j in (1,2)]
            change=max(changes)
            prior_min=[min(r[i] for r in earlier) for i in range(11)];prior_max=[max(r[i] for r in earlier) for i in range(11)]
            recent_min=[min(r[i] for r in recent) for i in range(11)];recent_max=[max(r[i] for r in recent) for i in range(11)]
            spread=[recent_max[i]-recent_min[i] for i in range(11)]
            envelope_change=max(max(abs(recent_min[i]-prior_min[i]),abs(recent_max[i]-prior_max[i]))/scales[i] for i in range(11))
            # Bounds apply to changes in the native limit cycle, rather than
            # its amplitude. A governor is allowed to oscillate physically;
            # its mean and envelope must be stable across independent windows.
            if change<=7e-4 and envelope_change<=1e-3:
                rows=earlier+recent;sample_frames=len(rows)
                mean=[sum(r[i] for r in rows)/sample_frames for i in range(11)]
                overspeed=[i for i,e in enumerate(p['engines']) if max(r[i] for r in engine_speeds[-sample_frames:])>e['properties']['omega_limit']*(1.+1e-6)]
                stopped=[i for i in range(len(p['transmissions'])) if min(r[i] for r in shaft_speeds[-sample_frames:])<1.]
                return dict(state=state,force=mean[:3],moment=mean[3:6],angular_momentum=mean[6:9],wash=mean[9:],
                    converged=True,feasible=not stopped,overspeed_engines=overspeed,stopped_shafts=stopped,
                    period_frames=None,cycle_samples=rows,stationarity=dict(method='bounded native limit-cycle mean',
                        window_seconds=sample_frames*dt,comparison_window_seconds=window*dt,
                        window_relative_changes=changes,output_range=spread,
                        envelope_relative_change=envelope_change,mean_relative_change_limit=7e-4,
                        envelope_relative_change_limit=1e-3),
                    window_relative_change=change,simulated_seconds=(index+1)*dt)
            if aircraft_residual_scales is not None and (index+1)*dt>=20.:
                from prop_cycle import aircraft_window
                certificate=aircraft_window(outputs,max(window,round(6./dt)),*aircraft_residual_scales)
                if certificate is not None:
                    count=2*certificate['frames'];rows=outputs[-count:]
                    mean=[sum(r[i] for r in rows)/count for i in range(11)]
                    overspeed=[i for i,e in enumerate(p['engines']) if max(r[i] for r in engine_speeds[-count:])>e['properties']['omega_limit']*(1.+1e-6)]
                    stopped=[i for i in range(len(p['transmissions'])) if min(r[i] for r in shaft_speeds[-count:])<1.]
                    return dict(state=state,force=mean[:3],moment=mean[3:6],angular_momentum=mean[6:9],wash=mean[9:],
                        converged=True,feasible=not stopped,overspeed_engines=overspeed,stopped_shafts=stopped,
                        period_frames=None,cycle_samples=rows,
                        stationarity=dict(method='bounded native limit-cycle mean',
                            window_alignment='aircraft force/moment error budget',window_seconds=count*dt,
                            comparison_window_seconds=certificate['frames']*dt,
                            **{k:v for k,v in certificate.items() if k!='frames'}),
                        window_relative_change=max(certificate['changes']),simulated_seconds=(index+1)*dt)
    if require_cycle:
        # The requested recurrence/stationarity certificate did not close.
        # Keep diagnostic forces from the existing final window, without
        # advancing an engine that has already exhausted the cycle budget.
        start=max(0,len(outputs)-max(1,round(20./dt)))
        rows=outputs[start:];mean=[sum(r[i] for r in rows)/len(rows) for i in range(11)]
        overspeed=[i for i,e in enumerate(p['engines']) if max(r[i] for r in engine_speeds[start:])>e['properties']['omega_limit']*(1.+1e-6)]
        stopped=[i for i in range(len(p['transmissions'])) if min(r[i] for r in shaft_speeds[start:])<1.]
        return dict(state=state,force=mean[:3],moment=mean[3:6],angular_momentum=mean[6:9],wash=mean[9:],
            converged=False,feasible=False,overspeed_engines=overspeed,stopped_shafts=stopped,
            period_frames=None,cycle_samples=None,window_relative_change=None,simulated_seconds=len(outputs)*dt)
    result=settle(p,velocity,height,body_omega,cg,dt,nitro,state=state,warm_seconds=0.,torque_gyro=torque_gyro)
    result['simulated_seconds']+=len(outputs)*dt;result['period_frames']=None;result['cycle_samples']=None
    return result


def settle(p,velocity,height,body_omega=(0.,0.,0.),cg=(0.,0.,0.),dt=1/48,nitro=0.,
           state=None,controls=None,warm_seconds=40.,sample_seconds=20.,max_windows=4,
           relative_tolerance=2e-4,torque_gyro=True):
    dt=f32(dt)
    if state is None:state=initial_state(p,velocity,height,nitro=nitro,**(controls or {}))
    else:state=copy.deepcopy(state)
    warm=max(0,round(warm_seconds/dt));sample=max(1,round(sample_seconds/dt))
    if warm:state,_=run_frames(p,state,warm,velocity,height,body_omega,cg,dt,nitro,torque_gyro=torque_gyro)
    windows=[];error=math.inf;converged=False
    for _ in range(max_windows):
        state,window=run_frames(p,state,sample,velocity,height,body_omega,cg,dt,nitro,torque_gyro=torque_gyro);windows.append(window)
        if len(windows)>1:
            a,b=windows[-2]['mean'],window['mean']
            scales=[max(100.,abs(a[i]),abs(b[i])) if i<9 else max(1.,abs(a[i]),abs(b[i])) for i in range(11)]
            error=max(abs(x-y)/scale for x,y,scale in zip(a,b,scales))
            if error<=relative_tolerance:converged=True;break
    mean=windows[-1]['mean'];last=windows[-1]
    overspeed=[i for i,(e,maximum) in enumerate(zip(p['engines'],last['engine_max_omega'])) if maximum>e['properties']['omega_limit']*(1.+1e-6)]
    stopped=[i for i,x in enumerate(last['shaft_min_omega']) if x<1.]
    return dict(state=state,force=mean[:3],moment=mean[3:6],angular_momentum=mean[6:9],wash=mean[9:],
                converged=converged,window_relative_change=error,overspeed_engines=overspeed,stopped_shafts=stopped,
                feasible=converged and not stopped,windows=windows,
                simulated_seconds=(warm+sample*len(windows))*dt)
