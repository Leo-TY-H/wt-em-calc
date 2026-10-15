import copy
from collections import OrderedDict
from prop_catalog import assets

from prop_steady import retained_key
from component_assembly import f32
from em_cancellation import check as check_cancel
from em_manual_controls import propeller_commands


class PropellerEnsemble:
    def __init__(self,name,model,mass,config):
        self.name=name;self.properties=assets(name)[0];self.mass=mass;self.config=config
        self.dt=1/config['timestep_hz'];self.fixed_controls=None;self._warm=None;self.force_canonical=False
        self.prefer_short_search=False;self.search_cycle_seconds=20.;self._reference=None
        self._certificates=OrderedDict();self._certificate_frames=0
        self._pending=OrderedDict()
        self.quasi_steady=config['engine_control_mode']=='quasi_steady'
        self.automatic=config['engine_control_mode'] in ('automatic','quasi_steady')
        self.automatic_controls=dict(propeller_commands(self.properties,config),
            gears=[0]*len(self.properties['engines']),
            throttle=config['throttle'],afterburner=config['afterburner'],engine_control_mode=config['engine_control_mode'])
        from prop_quasisteady import REVISION
        from prop_governor_branch import REVISION as branch_revision


        self.properties=dict(self.properties,engines=[dict(e,properties=dict(e['properties'],amplitude=[0.,0.,0.]))
            for e in self.properties['engines']])
        self.summary=dict(engine_count=sum(e['family']!=3 for e in self.properties['engines']),
            policy='Quasi-steady propulsion; ideal RPM governor with blade-pitch stops; radiators closed',
            nozzle_policy='Neutral nozzle controls; optional rocket boosters off',
            fuel_policy='Initial internal fuel and boost consumables frozen',
            model_revision=REVISION,
            governor_branch_revision=branch_revision,
            governor_branch_policy='First restoring torque balance reached from minimum blade pitch; transient hysteresis omitted',
            engine_health_policy='Intact engines; no thermal or overspeed damage; balanced steady shaft torque',
            optimization='Instantaneous governor and compressor regulation; steady induced flow; automatic mixture and stage selection',
            approximation='Instantaneous governor and steady wake with continuous blade polars; nominal healthy engine torque; propulsion oscillations and transient peak loads are omitted')
        if config.get('manual_engine_controls'):
            self.summary.update(policy='Quasi-steady propulsion; selected manual controls and radiator drag',
                optimization='Selected propeller command; automatic mixture and compressor; no manual search',
                engine_health_policy='Intact engines; RPM-limit diagnostics only; thermal endurance not evaluated')

        if config.get('trim_solver_mode') == 'stationary':
            self.summary.update(policy='Stationary requested RPM; fixed/manual pitch retains unknown speed',
                governor_branch_policy='Lowest-pitch admitted restoring root, otherwise lowest-pitch admitted root; equal-pitch ties retained',
                governor_branch_revision='stationary-any-power-v1',
                approximation='Legacy source polars retained including blade stall; forward flow; any absorbed-power sign; global selection unresolved')


    def with_controls(self,controls):
        view=copy.copy(self);view.fixed_controls=copy.deepcopy(controls)


        view._reference=(copy.deepcopy(self._warm))
        view._warm=None
        view._certificates=OrderedDict();view._certificate_frames=0
        view._pending=OrderedDict()
        return view

    def condition(self,velocity,omega,flaps=0.,speed=None,canonical=False,cycle_seconds=60.,certify_stationary=False,preserve_cycle_samples=False):
        check_cancel()


        velocity=tuple(map(f32,velocity));omega=tuple(map(f32,omega))
        independent=bool(canonical or self.force_canonical)


        key=(velocity,omega,(None),(None),
             independent,bool(certify_stationary and not self.automatic))


        start=self._reference if independent else self._warm
        pending_key=(key,retained_key(start) if start is not None else None)
        result=self._certificates.get(key)
        if result is None:
            pending=(self._pending.pop(pending_key,None))
            if self.quasi_steady and pending and pending.get('result') is not None:
                result=pending['result']
                from em_trim_work import charge
                charge('engine_failed_attempt_reuse')
            else:
                result=self._condition(velocity,omega,flaps,speed,canonical,cycle_seconds,certify_stationary,preserve_cycle_samples,
                                       resume=pending['state'] if pending else None,
                                       observation=pending['observation'] if pending and (independent or preserve_cycle_samples) else None)
            observation=result.pop('_observation',None)
            if pending:
                result['continued_simulated_seconds']=pending['elapsed']+result['simulated_seconds']
            if not result['converged'] and self.automatic:


                self._pending[pending_key]=dict(state=result['state'],observation=observation,
                    elapsed=result.get('continued_simulated_seconds',result['simulated_seconds']))
                self._pending[pending_key]['result']=result
                while (len(self._pending)>128 or
                       sum(len((p['observation'] or {}).get('outputs',[])) for p in self._pending.values())>16384):
                    self._pending.popitem(last=False)
            if result['converged']:


                self._certificates[key]=result
                self._certificate_frames+=len(result.get('cycle_samples') or [])
                while len(self._certificates)>128 or self._certificate_frames>16384 and len(self._certificates)>1:
                    _,old=self._certificates.popitem(last=False)
                    self._certificate_frames-=len(old.get('cycle_samples') or [])
        else:self._certificates.move_to_end(key)


        if result['converged']:
            self._warm=result['state']
            if (result.get('stationarity') or {}).get('method') in ('bounded native limit-cycle mean','repeating native output cycle'):
                self.prefer_short_search=True
                if (result.get('stationarity') or {}).get('window_alignment')=='aircraft force/moment error budget':
                    self.search_cycle_seconds=max(20.,min(180.,result.get('observation_seconds',result['simulated_seconds'])))
        if (result.get('stationarity') or {}).get('method') in ('bounded native limit-cycle mean','repeating native output cycle') and not (independent or preserve_cycle_samples):


            return dict(result,cycle_samples=None)
        return result

    def reset_search(self):
        self._warm=copy.deepcopy(getattr(self,'_task_seed',None));self.prefer_short_search=False;self.search_cycle_seconds=20.
        self.clear_cached_conditions()

    def clear_cached_conditions(self):
        self._certificates.clear();self._certificate_frames=0
        self._pending.clear()

    def _condition(self,velocity,omega,flaps=0.,speed=None,canonical=False,cycle_seconds=60.,certify_stationary=False,preserve_cycle_samples=False,resume=None,observation=None):
        check_cancel()
        if self.config.get('trim_solver_mode') == 'stationary':
            raise ValueError('Stationary propulsion requires a joint torque/wake candidate; legacy pitch-stop fallback is disabled')
        from prop_equilibrium import solve
        independent=canonical or self.force_canonical
        warm=self._reference if independent else self._warm
        if resume is not None:warm=resume
        return solve(self.properties,velocity,self.config['altitude_m'],omega,self.mass['cog'],self.dt,
            self.mass['nitro_mass'],self.fixed_controls or self.automatic_controls,self.config['torque_gyro'],state=warm)


