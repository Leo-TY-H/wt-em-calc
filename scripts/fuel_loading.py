"""Native tank-priority fuel placement, 101990060, with explicit presence.

Properties come from the original mass loader/geometry producer. High priorities
fill first; tanks within one priority receive equal fractions of capacity.
allocate() returns the allocator's intermediate tank-only system total.
initial() also applies the running fuel update's reservoir-inclusive total.
"""
from component_assembly import f32,add,sub,mul


def allocate(tanks,systems,state,system,amount,internal=True,external=False,clamp=True):
    s=dict(tanks=list(state.get('tanks',[0.]*len(tanks))),
           systems=[list(x) for x in state.get('systems',[[0.,0.,0.] for _ in systems])],
           present=list(state.get('present',[True]*len(tanks))))
    p=systems[system];amount=f32(amount);s['systems'][system][:2]=[0.,0.]
    if internal:
        reservoir=min(f32(p['reservoir_capacity']),amount)
        s['systems'][system][2]=reservoir;amount=sub(amount,reservoir)
    total=external_total=0.
    count=int(p['priority_count'])
    for priority in range(count-1,-1,-1):
        rows=[i for i,t in enumerate(tanks) if t['system']==system and t['priority']==priority and s['present'][i]]
        if count==1:
            capacity=add(sub(p['capacity'],p['external_capacity']) if internal else 0.,p['external_capacity'] if external else 0.)
        else:
            capacity=0.
            for i in rows:
                if external if tanks[i]['external'] else internal:capacity=add(capacity,tanks[i]['capacity'])
        ratio=f32(amount/capacity) if abs(capacity)>f32(4e-19) else 0.
        if clamp or priority!=0:
            ratio=min(1.,max(0.,ratio));amount=max(sub(amount,capacity),0.)
        else:amount=0.
        for i in rows:
            tank=tanks[i]
            if external if tank['external'] else internal:s['tanks'][i]=mul(tank['capacity'],ratio)
            total=add(total,s['tanks'][i])
            if tank['external']:external_total=add(external_total,s['tanks'][i])
            s['systems'][system][:2]=[total,external_total]
    return s


def initial(tanks,systems,fraction):
    """Clean internal selection, ready for running flight with fuel held fixed.

    101990060 separates the feed reservoir before filling tanks. The running
    update 10199a210 sums tanks in ascending priority and adds the reservoir at
    10199a5ea. Reproduce its zero-consumption limit before evaluating mass; the
    allocator's intermediate total alone undercounts fuel by the reservoir.
    """
    state=dict(tanks=[0.]*len(tanks),systems=[[0.,0.,0.] for _ in systems],
               present=[not t['external'] for t in tanks])
    for i,p in enumerate(systems):
        amount=mul(sub(p['capacity'],p['external_capacity']),f32(fraction))
        state=allocate(tanks,systems,state,i,amount)
    for i,p in enumerate(systems):
        total=0.
        for priority in range(int(p['priority_count'])):
            for j,t in enumerate(tanks):
                if (t['system']==i and t['priority']==priority and
                        state['present'][j] and state['tanks'][j]>0.):
                    total=add(total,state['tanks'][j])
        state['systems'][i][0]=add(max(state['systems'][i][2],0.),total)
    return dict(fuel_by_tank=state['tanks'],fuel_by_system=[x[0] for x in state['systems']],
                reservoir=[x[2] for x in state['systems']])
