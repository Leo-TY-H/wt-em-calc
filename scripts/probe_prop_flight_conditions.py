"""Research-only sensitivities: turn sense, sideslip and measured additional mass."""
import inspect
import json
from pathlib import Path
import em_solver as em
from analyze_prop_flights import OUT, brief, G, apply_recorded_mass
from prop_catalog import mass_state


def matched_solver(row):
    name=row['file'].split('-2026_')[0]
    cfg=em.settings(dict(aircraft=[name],fuel_percent=30.,altitude_m=row['means']['Altitude, m'],
                         instructor=row['instructor']))
    solver=em.TrimSolver(name,cfg)
    extra=apply_recorded_mass(solver,row)
    return solver,extra


def main():
    rows=json.loads((OUT/'telemetry.json').read_text())
    original=em.turn_geometry
    code=inspect.getsource(original).replace('def turn_geometry(', 'def mirrored_geometry(')
    code=code.replace('(alpha_deg,bank_deg)', '(alpha_deg,-bank_deg)')
    code=code.replace('turn_rate=float(G)', 'turn_rate=-float(G)')
    scope=dict(em.__dict__);exec(code,scope);mirrored=scope['mirrored_geometry']
    results=[]
    for row in rows:
        for direction in [1,-1]:
            em.turn_geometry=original if direction==1 else mirrored
            for beta in [0.,row['means']['AoS, deg'],-row['means']['AoS, deg']]:
                solver,extra=matched_solver(row)
                # Cached body beta differs slightly from this attitude
                # parameter at nonzero alpha; retain both in the evidence.
                solver=solver.at_sideslip(beta)
                p=solver.solve(row['means']['TAS, km/h'],row['equivalent_level_load_g'],exhaustive=False,detailed=True)
                value=p.pop('_detail')
                record=dict(file=row['file'],direction=direction,beta_attitude=beta,
                    extra_mass_kg=extra,mass_policy='30% fuel; additional recorded mass at nominal FM CG for sensitivity only',
                    point=brief(p),body_y_load_g=value['result']['force'][1]/(solver.mass['mass']*G))
                if row['instructor']:
                    high=solver.solve(p['speed_kmh'],p['load_g']*1.1,p['solution'],exhaustive=False)
                    edge=solver.boundary(p['speed_kmh'],p,high,continuation=False)
                    record['boundary']=brief(edge)
                results.append(record)
                (OUT/'condition-sensitivity.json').write_text(json.dumps(results,indent=2)+'\n')
                print(row['file'],direction,beta,'actual beta',p['sideslip_deg'],'alpha',round(p['alpha_deg'],4),
                      'Ps',round(p['ps_mps'],4),'thrust',round(p['engine_force_n'][0]/G,3),'valid',p['valid'],
                      'commands',[round(x,4) for x in p['commands']],flush=True)
    em.turn_geometry=original


if __name__=='__main__':main()
