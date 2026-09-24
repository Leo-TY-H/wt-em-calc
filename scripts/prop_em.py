"""Propulsion and geometry-backed mass interfaces for EM trim integration.

The initial control search is a propulsion seed. Coupled aircraft refinement
must evaluate/replay its selected settings before advertising optimum EM.
"""
import copy,math,os
from collections import OrderedDict
from functools import lru_cache
from prop_catalog import assets
from prop_optimize import candidates
from prop_steady import initial_state,settled_cycle
from propulsion_general import target_omega
from component_assembly import f32
from em_cancellation import check as check_cancel


class PropellerEnsemble:
    def __init__(self,name,model,mass,config):
        self.name=name;self.properties=assets(name)[0];self.mass=mass;self.config=config
        self.dt=1/config['timestep_hz'];self.fixed_controls=None;self._warm=None;self.force_canonical=False
        self.prefer_short_search=False;self.search_cycle_seconds=20.;self._reference=None
        self._certificates=OrderedDict();self._certificate_frames=0
        self._pending=OrderedDict()
        self.automatic=config['engine_control_mode']=='automatic'
        self.automatic_controls=dict(commands=[255]*len(self.properties['propellers']),
            automatic=[True]*len(self.properties['propellers']),gears=[0]*len(self.properties['engines']),
            throttle=config['throttle'],afterburner=config['afterburner'],engine_control_mode='automatic')
        self.summary=dict(engine_count=sum(e['family']!=3 for e in self.properties['engines']),
            policy=('Native automatic engine controls; radiators closed' if self.automatic else
                    'Native settled propeller/engine states; radiators closed; best legal control search'),
            nozzle_policy='Neutral nozzle controls; optional rocket boosters off',
            fuel_policy='Initial internal fuel and boost consumables frozen',
            engine_health_policy='Intact engines; no thermal or overspeed damage; native governor and shaft dynamics',
            optimization=('Automatic governor, mixture and compressor; no manual search' if self.automatic else
                'Discrete stage/mode multistart and command refinement using re-trimmed Ps; global optimum not certified'))

    @lru_cache(maxsize=256)
    def seed_controls(self,speed):
        return candidates(self.properties,[speed,0.,0.],self.config['altitude_m'],cg=self.mass['cog'],dt=self.dt,
            nitro=self.mass['nitro_mass'],throttle=self.config['throttle'],afterburner=self.config['afterburner'],exhaustive=False,torque_gyro=self.config['torque_gyro'])

    def with_controls(self,controls):
        view=copy.copy(self);view.fixed_controls=copy.deepcopy(controls)
        # Numerical corrections must evaluate a reproducible native mean.
        # Reuse one fixed, already settled state for every trial instead of
        # restarting a healthy drivetrain from rest at each Jacobian entry.
        # The same recurrence/mean certificate and complete phase replay apply.
        view._reference=copy.deepcopy(self._warm) if self.automatic else None
        view._warm=None
        view._certificates=OrderedDict();view._certificate_frames=0
        view._pending=OrderedDict()
        return view

    def condition(self,velocity,omega,flaps=0.,speed=None,canonical=False,cycle_seconds=60.,certify_stationary=False,preserve_cycle_samples=False):
        check_cancel()
        # The propulsion owner converts both vectors to native float32 before
        # any engine or propeller equation reads them. Solver bank/rate values
        # arrive as doubles; sub-ulp Newton corrections must not restart the
        # same native trajectory under a different Python dictionary key.
        velocity=tuple(map(f32,velocity));omega=tuple(map(f32,omega))
        independent=bool(canonical or self.force_canonical)
        # Automatic propulsion depends on the actual rounded body flow/rates.
        # Plot speed and flap coordinates select no automatic engine control;
        # different display coordinates with identical native inputs therefore
        # have exactly the same engine calculation within this branch owner.
        key=(velocity,omega,None if self.automatic else flaps,None if self.automatic else speed,
             independent,bool(certify_stationary and not self.automatic))
        result=self._certificates.get(key)
        if result is None:
            pending=self._pending.pop(key,None) if not independent and self.automatic else None
            result=self._condition(velocity,omega,flaps,speed,canonical,cycle_seconds,certify_stationary,preserve_cycle_samples,
                                   resume=pending['state'] if pending else None)
            if pending:
                result['continued_simulated_seconds']=pending['elapsed']+result['simulated_seconds']
            if not result['converged'] and not independent and self.automatic:
                # A settling budget is not a physical engine failure. If the
                # exact condition is requested again, continue its native
                # trajectory instead of restarting the same slow transient.
                # A fresh complete recurrence/mean check is still required.
                self._pending[key]=dict(state=result['state'],elapsed=result.get('continued_simulated_seconds',result['simulated_seconds']))
                if len(self._pending)>128:self._pending.popitem(last=False)
            if result['converged']:
                # Search and final validation use the same native certificate.
                # The observation budget and whether the caller needs phases
                # cannot invalidate an already completed equilibrium. Retain
                # its actual phase frames instead of settling the identical
                # condition again with a different averaging phase.
                self._certificates[key]=result
                self._certificate_frames+=len(result.get('cycle_samples') or [])
                while len(self._certificates)>128 or self._certificate_frames>16384 and len(self._certificates)>1:
                    _,old=self._certificates.popitem(last=False)
                    self._certificate_frames-=len(old.get('cycle_samples') or [])
        else:self._certificates.move_to_end(key)
        # Cache hits are continuation steps too. Restore the returned state so
        # later trials cannot inherit whichever unrelated cache miss ran last.
        if result['converged']:
            self._warm=result['state']
            if (result.get('stationarity') or {}).get('method') in ('bounded native limit-cycle mean','repeating native output cycle'):
                self.prefer_short_search=True
                if (result.get('stationarity') or {}).get('window_alignment')=='aircraft force/moment error budget':
                    self.search_cycle_seconds=max(20.,min(180.,result['simulated_seconds']))
        if (result.get('stationarity') or {}).get('method') in ('bounded native limit-cycle mean','repeating native output cycle') and not (independent or preserve_cycle_samples):
            # A cheap search residual may use the mean, but must not destroy
            # the native phase record needed by the nonlinear final consumer.
            return dict(result,cycle_samples=None)
        return result

    def reset_search(self):
        self._warm=copy.deepcopy(getattr(self,'_task_seed',None));self.prefer_short_search=False;self.search_cycle_seconds=20.
        self.clear_cached_conditions()

    def clear_cached_conditions(self):
        self._certificates.clear();self._certificate_frames=0
        self._pending.clear()

    def _condition(self,velocity,omega,flaps=0.,speed=None,canonical=False,cycle_seconds=60.,certify_stationary=False,preserve_cycle_samples=False,resume=None):
        check_cancel()
        # Reuse the observation length actually required by this native
        # governor, rather than repeatedly timing out a fixed short window.
        cycle_seconds=max(cycle_seconds,self.search_cycle_seconds)
        if speed is None:speed=math.sqrt(sum(x*x for x in velocity))
        # The explicit fixed control set is used during coupled optimization.
        # An unfixed instance supplies a reproducible nominal-flight seed.
        controls=self.fixed_controls or (self.automatic_controls if self.automatic else self.seed_controls(speed)['controls'])
        independent=canonical or self.force_canonical
        warm=(self._reference if independent and self._reference is not None else
              None if independent else self._warm)
        if resume is not None:warm=resume
        cold=warm is None
        initializer=None
        if self.automatic and os.environ.get('WT_EM_PROP_FIXEDPOINT','1')!='0':
            from prop_fixedpoint import propose
            proposal_seed=(warm if warm is not None else initial_state(self.properties,velocity,
                self.config['altitude_m'],nitro=self.mass['nitro_mass'],**controls))
            proposal=propose(self.properties,proposal_seed,velocity,self.config['altitude_m'],omega,
                             self.mass['cog'],self.dt,self.mass['nitro_mass'],self.config['torque_gyro'],allow_long_map=False)
            if proposal is not None:warm,initializer=proposal
        def governed_start():
            start=initial_state(self.properties,velocity,self.config['altitude_m'],
                nitro=self.mass['nitro_mass'],**controls)
            for prop,state in zip(self.properties['propellers'],start['propellers']):
                if prop['properties']['governor']!=0:
                    state['pitch']=state['governor_pitch']=prop['properties']['pitch_max']
            return start
        result=settled_cycle(self.properties,velocity,self.config['altitude_m'],omega,self.mass['cog'],self.dt,
            self.mass['nitro_mass'],controls,max_seconds=max(60.,cycle_seconds) if independent else cycle_seconds,
            state=warm,require_cycle=True,allow_stationary=True,torque_gyro=self.config['torque_gyro'],
            # Native automatic governors may settle onto a bounded float32
            # limit cycle whose retained state never repeats exactly. Apply
            # the same convergence certificate at every flight condition so
            # this cannot become an aircraft-specific hole in the EM surface.
            allow_stationary_mean=self.automatic or certify_stationary,cold_start=cold,
            aircraft_residual_scales=(self.mass['mass']*9.8100004196167,self.mass['inertia'],self.config['torque_gyro']) if self.automatic else None)
        if initializer is not None:result['numerical_initializer']=initializer
        if (self.automatic and os.environ.get('WT_EM_PROP_FIXEDPOINT','1')!='0' and
                (not result['converged'] or os.environ.get('WT_EM_PROP_REFINE_MEAN','0')!='0' and
                 (result.get('stationarity') or {}).get('method')=='bounded native limit-cycle mean')):
            # Recover a trajectory that has not settled. A completed native
            # cycle-mean certificate already permits the full aircraft replay;
            # searching again for a stationary root repeats engine work and
            # can change the averaging phase. Preserve the former extra search
            # behind a diagnostic switch. Final aircraft closure and phase
            # uncertainty checks still apply to every accepted point.
            proposal=propose(self.properties,result['state'],velocity,self.config['altitude_m'],omega,
                self.mass['cog'],self.dt,self.mass['nitro_mass'],self.config['torque_gyro'])
            if proposal is not None:
                candidate=settled_cycle(self.properties,velocity,self.config['altitude_m'],omega,self.mass['cog'],self.dt,
                    self.mass['nitro_mass'],controls,max_seconds=cycle_seconds,state=proposal[0],require_cycle=True,
                    allow_stationary=True,torque_gyro=self.config['torque_gyro'],allow_stationary_mean=True,
                    aircraft_residual_scales=(self.mass['mass']*9.8100004196167,self.mass['inertia'],self.config['torque_gyro']))
                if candidate['converged']:
                    candidate['numerical_initializer']=proposal[1]
                    candidate['simulated_seconds']+=result['simulated_seconds']
                    result=candidate
        if self.automatic:
            governed={link['index'] for transmission in self.properties['transmissions']
                if any(self.properties['propellers'][p['index']]['properties']['governor']!=0
                       for p in transmission['propellers']) for link in transmission['engines']}
            def command_error(candidate):
                errors=[]
                for i in governed:
                    state=candidate['state']['engines'][i]
                    target=target_omega(self.properties['engines'][i]['properties'],state,self.mass['nitro_mass'])
                    if target>0:errors.append(abs(state['omega']-target)/target)
                return max(errors,default=0.)
            error=command_error(result)
            if governed and result['force'][0]<0. and controls['throttle']>0.:
                # A minimum-pitch start can settle on a windmilling branch
                # even when the automatic governor has a stable commanded-RPM
                # solution. Try the opposite legal pitch endpoint. Propagate
                # every native frame. Prefer a powered branch only when it
                # also satisfies the governor command at least as well. Both
                # roots retain the game's forces, limits and actual RPM.
                start=governed_start()
                alternate=settled_cycle(self.properties,velocity,self.config['altitude_m'],omega,
                    self.mass['cog'],self.dt,self.mass['nitro_mass'],controls,max_seconds=60.,state=start,
                    require_cycle=True,allow_stationary=True,torque_gyro=self.config['torque_gyro'],
                    allow_stationary_mean=True,cold_start=True)
                if (alternate['converged'] and alternate['feasible'] and alternate['force'][0]>0.
                        and command_error(alternate)<=max(.001,error)):
                    result=alternate
                    result['initialization']='native automatic RPM branch from maximum legal blade pitch'
        stationarity=result.get('stationarity') or {}
        bounded_mean=stationarity.get('method') in ('bounded native limit-cycle mean','repeating native output cycle')
        if result['converged']:self._warm=result['state']
        # Once a native output certificate is needed, keep subsequent search
        # trials bounded. Final points still get complete native phase checks.
        if result['converged'] and bounded_mean:self.prefer_short_search=True
        result['controls']=controls
        result['canonical_initialization']=independent or cold
        if independent and self._reference is not None:
            result['initialization']='fixed settled native state for numerical correction'
        return result
