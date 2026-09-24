"""Intersect the current Instructor permission boundary with Ps=0 for recorded conditions."""
import json
from scipy.optimize import brentq
from analyze_prop_flights import OUT, brief, apply_recorded_mass
from probe_prop_flight_conditions import matched_solver
from em_sampling import worker_solver, sample_column


def main():
    results=[]
    for row in json.loads((OUT/'telemetry.json').read_text()):
        if not row['instructor']:continue
        solver,_=matched_solver(row);observed=row['means']['TAS, km/h'];memo={}
        def at(speed):
            if speed not in memo:
                load=row['equivalent_level_load_g']*(speed/observed)**2
                low=solver.solve(speed,max(1.,load*.85),exhaustive=False)
                high=solver.solve(speed,load*1.18,low['solution'],exhaustive=False)
                edge=solver.boundary(speed,low,high,continuation=False)
                if not edge:
                    encoded=json.dumps(solver.config)
                    cached=worker_solver(solver.name,encoded)
                    if cached.mass['mass']!=solver.mass['mass']:apply_recorded_mass(cached,row)
                    edge=sample_column((solver.name,encoded,speed,None))['boundary']
                if not edge or not edge['valid'] or edge['envelope_limit']['kind']!='Instructor AoA':
                    raise ValueError(('No verified permission boundary',speed))
                memo[speed]=edge
                print(row['file'],speed,edge['ps_mps'],edge['turn_dps'],flush=True)
            return memo[speed]['ps_mps']
        root=brentq(at,observed*.85,observed*1.05,xtol=.005)
        at(root)
        results.append(dict(file=row['file'],boundary_ps_zero=brief(memo[root]),
                            probes=[brief(memo[v]) for v in sorted(memo)]))
        (OUT/'instructor-sustained.json').write_text(json.dumps(results,indent=2)+'\n')


if __name__=='__main__':main()
