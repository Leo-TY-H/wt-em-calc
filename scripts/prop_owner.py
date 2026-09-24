"""Single connected propeller owner: original double aggregation semantics."""
from component_assembly import sub
from propulsion_model import step

def owner_step(*args,**kw):
    r=step(*args,**kw);o=r['outputs']
    r.update(aggregate_force=o[:3],aggregate_moment=o[3:6],engine_angular_momentum=o[6:9],
             propeller_force=o[:3],engine_wash=[o[9],sub(o[10],o[11])],shake=r['prop']['outputs'][29])
    return r
