"""Comparison identities are separate from physical aircraft/cache identities."""
from em_data import clone
import re
import time

COLORS=('#38c9d7','#ffa66b','#b79aff','#91d477','#ee8eb6','#f0d367','#79a7fa','#cfb296')
ENTRY_ID=re.compile(r'entry_[A-Za-z0-9_-]{1,48}\Z')


def entry_settings(values):
    from em_solver import settings,AIRCRAFT,AIRCRAFT_SETTINGS
    entries=values['entries']
    if not isinstance(entries,list) or not 1<=len(entries)<=len(COLORS):
        raise ValueError('Add between one and eight aircraft entries')
    if values.get('compare_instructor',False):
        raise ValueError('Set Instructor separately for each aircraft entry')
    normalized=[];ids=set()
    for entry in entries:
        if not isinstance(entry,dict) or set(entry)!={'id','aircraft_id','settings'}:
            raise ValueError('Each entry requires id, aircraft_id and settings')
        identity=entry['id'];name=entry['aircraft_id'];condition=entry['settings']
        if not isinstance(identity,str) or not ENTRY_ID.fullmatch(identity) or identity in ids:
            raise ValueError('Aircraft entry IDs must be unique entry_ identifiers')
        if not isinstance(name,str) or name not in AIRCRAFT:
            raise ValueError('Unknown aircraft in entry')
        if not isinstance(condition,dict) or set(condition)-AIRCRAFT_SETTINGS:
            raise ValueError('Unknown aircraft condition in entry')
        # Reuse all physical validation, including flap/sweep and Instructor limits.
        resolved=settings(dict(values,**dict(condition,entries=None,aircraft=[name],
                                             aircraft_settings={},compare_instructor=False)))
        normalized.append(dict(id=identity,aircraft_id=name,
                               settings={k:resolved[k] for k in sorted(AIRCRAFT_SETTINGS)}))
        ids.add(identity)
    # Shared settings are validated with a valid entry's physical conditions.
    # Physical settings in this outer object are inert; each entry is explicit.
    result=dict(resolved,entries=normalized,aircraft=list(dict.fromkeys(e['aircraft_id'] for e in normalized)))
    return result


def compute_entries(config,progress=None,cancelled=None,preview=None):
    from em_solver import compute,AIRCRAFT
    start=time.monotonic();entries=config['entries'];completed={};runs=[]
    # Keep existing parallel two-aircraft solves. A repeated model starts a new
    # batch, with its own physical settings and the existing exact-result cache.
    batches=[]
    for index,entry in enumerate(entries):
        if not batches or len(batches[-1])==2 or any(e['aircraft_id']==entry['aircraft_id'] for _,e in batches[-1]):
            batches.append([])
        batches[-1].append((index,entry))
    def assemble(data,batch):
        rows=dict(completed)
        physical={a['id']:a for a in data['aircraft']}
        for index,entry in batch:
            if entry['aircraft_id'] not in physical:continue
            a=dict(physical[entry['aircraft_id']])
            condition=entry['settings']
            mode=('RB' if condition['instructor'] else 'SB') if condition['instructor']!=condition['torque_gyro'] else (
                f"Instructor {'on' if condition['instructor'] else 'off'} · torque/gyro {'on' if condition['torque_gyro'] else 'off'}")
            a.update(id=entry['id'],aircraft_id=entry['aircraft_id'],color=COLORS[index],
                     name=f"{AIRCRAFT[entry['aircraft_id']]['name']} · {index+1} · {mode}")
            rows[entry['id']]=a
        output=dict(data,settings=config,aircraft=[rows[e['id']] for e in entries if e['id'] in rows],
                    elapsed_s=time.monotonic()-start)
        output['speeds_kmh']=sorted({v for a in output['aircraft'] for v in
            ([c['speed_kmh'] for c in a['columns']] if 'columns' in a else data['speeds_kmh'])})
        return output
    for batch in batches:
        if cancelled and cancelled():raise InterruptedError('Calculation cancelled')
        work=dict(config,entries=None,aircraft=[e['aircraft_id'] for _,e in batch],
                  aircraft_settings={e['aircraft_id']:e['settings'] for _,e in batch})
        # Global physical defaults must not depend on another entry. Per-entry
        # overrides above carry every physical setting to the numerical solver.
        def report(p):
            total=max(1,p.get('total',1))
            progress(dict(p,done=len(completed)+len(batch)*p.get('done',0)/total,total=len(entries),
                          phase=f"Entries {batch[0][0]+1}–{batch[-1][0]+1} · "+p.get('phase','Solving'),
                          elapsed_s=time.monotonic()-start))
        run=compute(work,report if progress else None,cancelled,
                    (lambda data:preview(clone(assemble(data,batch)))) if preview else None)
        output=assemble(run,batch);runs.append(run)
        completed={a['id']:a for a in output['aircraft']}
        if preview and len(completed)<len(entries):preview(clone(dict(output,preview=True)))
    output['cached_aircraft']=[e['id'] for batch,run in zip(batches,runs) for _,e in batch
                               if e['aircraft_id'] in run.get('cached_aircraft',[])]
    output['cached_columns']=sum(r.get('cached_columns',0) for r in runs)
    return output
