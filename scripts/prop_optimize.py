"""Legal delivered-control candidates from complete installed drivetrain dynamics.

The independent transmission search maximizes force along the prescribed body
velocity. Its winner is an aircraft-trim seed, not a minimum-drag EM claim.
"""
import copy,itertools,math
from em_cancellation import check as check_cancel
from prop_steady import settled_cycle


def transmission_configuration(p,index):
    t=copy.deepcopy(p['transmissions'][index]);ei=[l['index'] for l in t['engines']];pi=[l['index'] for l in t['propellers']]
    q=dict(engines=[copy.deepcopy(p['engines'][i]) for i in ei],propellers=[copy.deepcopy(p['propellers'][i]) for i in pi],transmissions=[t])
    for i,e in enumerate(q['engines']):e['index']=i
    for i,r in enumerate(q['propellers']):r['index']=i
    for link in t['engines']:link['index']=ei.index(link['index'])
    for link in t['propellers']:link['index']=pi.index(link['index'])
    t['index']=0
    return q,ei,pi


def candidates(p,velocity,height,body_omega=(0.,0.,0.),cg=(0.,0.,0.),dt=1/48,nitro=0.,
               throttle=1.1,afterburner=True,exhaustive=True,torque_gyro=True):
    length=math.sqrt(sum(x*x for x in velocity));direction=[x/max(length,1e-12) for x in velocity]
    selections=[];counts=dict(evaluated=0,unresolved=0,overspeed=0,stopped=0)
    for index in range(len(p['transmissions'])):
        q,ei,pi=transmission_configuration(p,index);memo={};rows=[]
        gears=[range(len(e['properties']['stages'])) if e['properties']['manual_compressor'] else [0] for e in q['engines']]
        manual=all(r['manual'] or r['governor']==0 for r in q['propellers']) and any(r['governor']!=0 for r in q['propellers'])
        auto=all(r['automatic'] or r['governor']==0 for r in q['propellers']) and any(r['automatic'] for r in q['propellers'])
        def evaluate(gear,command,automatic):
            check_cancel()
            key=(gear,command,automatic)
            if key in memo:return memo[key]
            controls=dict(throttle=throttle,afterburner=afterburner,gears=list(gear),commands=[command]*len(pi),
                automatic=[automatic and r['automatic'] for r in q['propellers']])
            result=settled_cycle(q,velocity,height,body_omega,cg,dt,nitro,controls,torque_gyro=torque_gyro)
            score=sum(a*b for a,b in zip(result['force'],direction))
            row=dict(controls=controls,force_along_velocity=score,feasible=result['feasible'] and bool(result.get('cycle_samples')),
                converged=result['converged'],period_frames=result['period_frames'],
                overspeed_engines=result['overspeed_engines'],stopped_shafts=result['stopped_shafts'])
            counts['evaluated']+=1;counts['unresolved']+=not result['converged'] or not result.get('cycle_samples')
            counts['overspeed']+=bool(result['overspeed_engines']);counts['stopped']+=bool(result['stopped_shafts'])
            memo[key]=row;rows.append(row);return row
        for gear in itertools.product(*gears):
            if auto:evaluate(gear,255,True)
            if manual:
                for command in range(256) if exhaustive else list(range(0,256,16))+[255]:evaluate(gear,command,False)
                if not exhaustive:
                    for stride in [8,4,2,1]:
                        best=sorted((r for r in rows if r['feasible'] and r['controls']['gears']==list(gear)),key=lambda r:r['force_along_velocity'],reverse=True)[:3]
                        for r in best:
                            for command in [r['controls']['commands'][0]-stride,r['controls']['commands'][0]+stride]:
                                if 0<=command<=255:evaluate(gear,command,False)
            elif not auto:evaluate(gear,255,False)
        feasible=sorted((r for r in rows if r['feasible']),key=lambda r:r['force_along_velocity'],reverse=True)
        # An unresolved or stopped shaft is a masked operating point, not
        # a whole-job exception. Preserve the best diagnostic state when no
        # candidate can be accepted; no fallback is relabeled feasible.
        diagnostic=sorted(rows,key=lambda r:(r['converged'],not r['overspeed_engines'],
                          not r['stopped_shafts'],r['force_along_velocity']),reverse=True)
        selections.append(dict(transmission=index,engine_indices=ei,propeller_indices=pi,
            best=(feasible or diagnostic)[0],candidates=feasible,unresolved=[r for r in rows if not r['converged'] or r['period_frames'] is None]))
    controls=dict(throttle=throttle,afterburner=afterburner,commands=[255]*len(p['propellers']),
                  automatic=[r['automatic'] for r in p['propellers']],gears=[0]*len(p['engines']))
    for selection in selections:
        best=selection['best']['controls']
        for i,j in enumerate(selection['engine_indices']):controls['gears'][j]=best['gears'][i]
        for i,j in enumerate(selection['propeller_indices']):
            controls['commands'][j]=best['commands'][i];controls['automatic'][j]=best['automatic'][i]
    return dict(controls=controls,selections=selections,counts=counts,
        exhaustive_commands=exhaustive,scope=__doc__)
