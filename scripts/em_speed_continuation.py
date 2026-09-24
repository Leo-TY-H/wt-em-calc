"""Diagnostic speed/load continuation; not deployed without recovery evidence."""
import copy


def recover_from_speed(solver,speed_kmh,load,neighbors):
    """Continue verified equilibria through speed and load to the failed point.

    A neighboring speed supplies a seed, never a replacement value. Every
    continuation point closes the unchanged physical equations; only the final
    requested condition can be returned, with its normal feasibility checks.
    """
    candidates=[p for p in neighbors if p['valid'] and abs(p['speed_kmh']-speed_kmh)>1e-6]
    if not candidates:return None
    # Keep seeds from both sides of a Mach/polar transition and both nearby
    # load branches. Avoid spending retries on many nearly identical knots.
    selected=[]
    for side in [-1,1]:
        group=[p for p in candidates if (p['speed_kmh']-speed_kmh)*side>0]
        group.sort(key=lambda p:abs(p['speed_kmh']-speed_kmh)/max(1.,speed_kmh)+abs(p['load_g']-load)/max(1.,load))
        for p in group:
            if not any(abs(p['speed_kmh']-q['speed_kmh'])<1e-6 and abs(p['alpha_deg']-q['alpha_deg'])<.05 for q in selected):
                selected.append(p)
                if sum((q['speed_kmh']-speed_kmh)*side>0 for q in selected)>=2:break
    physical=copy.copy(solver);physical.config=dict(solver.config,instructor=False,aircraft_settings={})
    for source in selected:
        t=0.;step=.25;point=source;steps=0
        while t<1. and steps<64:
            nxt=min(1.,t+step);speed=source['speed_kmh']+(speed_kmh-source['speed_kmh'])*nxt
            n=source['load_g']+(load-source['load_g'])*nxt
            trial=physical.solve(speed,n,point['solution'],exhaustive=False);steps+=1
            if trial['converged'] and trial['stall_margin_deg']>=0:
                point=trial;t=nxt;step=min(.5,step*1.5)
            else:
                step*=.5
                if step<1/4096:break
        if t==1.:
            final=solver.solve(speed_kmh,load,point['solution'],exhaustive=False)
            if final['valid']:
                final['recovery_method']='balanced speed/load continuation'
                final['recovery_source']={k:source[k] for k in ['speed_kmh','load_g','alpha_deg']}
                final['recovery_steps']=steps
                return final
    return None
