"""Recorded held-pitch deceleration versus static Instructor angle permission."""
import json
import math
from pathlib import Path
import numpy as np
from em_solver import settings
from em_sampling import worker_solver, sample_column
from analyze_prop_flights import read, brief, apply_recorded_mass

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'analysis/prop-aoa-sweep'


def main():
    OUT.mkdir(exist_ok=True)
    path=next((ROOT/'FlightTestData').glob('*aoavsspeed.csv'))
    d=read(path);t=d['Time, s'];speed=d['TAS, km/h'];alpha=d['AoA, deg']
    assert np.all(np.diff(t)>0)
    intervals=[]
    for start in np.arange(0.,t[-1],2.):
        ix=np.where((t>=start)&(t<start+2.))[0]
        tt=t[ix]
        means={k:float(np.trapz(v[ix],tt)/(tt[-1]-tt[0])) for k,v in d.items()}
        intervals.append(dict(start_s=float(start),end_s=float(tt[-1]),means=means))
    plateau=(t>=8.)
    report=dict(file=path.name,samples=len(t),duration_s=float(t[-1]-t[0]),
        ranges={k:[float(v.min()),float(v.max())] for k,v in d.items()},
        intervals=intervals,plateau=dict(start_s=8.,mean_aoa_deg=float(np.trapz(alpha[plateau],t[plateau])/(t[-1]-t[plateau][0])),
            aoa_range_deg=[float(alpha[plateau].min()),float(alpha[plateau].max())],
            tas_range_kmh=[float(speed[plateau].min()),float(speed[plateau].max())]),
        model=[],notes=['Plateau is in AoA, not speed/altitude; no completed settling or pre-pull baseline in this clip.',
            'Measured total mass matched; additional mass at nominal CG is a diagnostic approximation, not validated ammo placement.'])
    for target in [650.,600.,550.,500.,450.,400.]:
        mask=abs(speed-target)<2.5
        means={k:float(np.mean(v[mask])) for k,v in d.items()}
        cfg=settings(dict(aircraft=['yak-3_france'],fuel_percent=30.,altitude_m=means['Altitude, m'],instructor=True,
                          speed_samples=9,load_samples=7,sep_tolerance_mps=1.))
        solver=worker_solver('yak-3_france',json.dumps(cfg));extra=apply_recorded_mass(solver,dict(means=means))
        v=means['TAS, km/h'];load=means['Load factor, g']/math.cos(math.radians(means['AoA, deg']))
        low=solver.solve(v,max(1.,load*.8),exhaustive=False)
        high=solver.solve(v,load*1.25,low['solution'],exhaustive=False)
        edge=solver.boundary(v,low,high,continuation=False)
        if not edge:edge=sample_column(('yak-3_france',json.dumps(cfg),v,None))['boundary']
        if edge is None:raise ValueError(('Unresolved boundary',target))
        row=dict(target_kmh=target,means=means,extra_mass_kg=extra,model_mass=solver.mass,
                 boundary=brief(edge),delta_aoa_deg=edge['alpha_deg']-means['AoA, deg'])
        report['model'].append(row)
        (OUT/'analysis.json').write_text(json.dumps(report,indent=2)+'\n')
        print(target,'recorded',means['AoA, deg'],'boundary',edge['alpha_deg'],edge['envelope_limit']['kind'],flush=True)
    # Retain the previous near-sea-level Instructor observations for context.
    prior=json.loads((ROOT/'analysis/prop-flight-comparison/telemetry.json').read_text())
    report['previous_instructor_runs']=[r for r in prior if r['file'].startswith('yak') and r['instructor']]
    (OUT/'analysis.json').write_text(json.dumps(report,indent=2)+'\n')
    plot(d,report)


def plot(d,report):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
    fig,axes=plt.subplots(2,2,figsize=(12,7.5))
    t=d['Time, s'];speed=d['TAS, km/h'];alpha=d['AoA, deg']
    ax=axes[0,0];ax.plot(speed,alpha,color='#25699f',lw=1.5,label='Recorded held-pitch turn')
    rows=sorted(report['model'],key=lambda r:r['means']['TAS, km/h'])
    ax.plot([r['means']['TAS, km/h'] for r in rows],[r['boundary']['alpha_deg'] for r in rows],
            'o--',color='#bd4933',label='Current modeled boundary')
    for i,r in enumerate(report['previous_instructor_runs']):
        ax.scatter(r['means']['TAS, km/h'],r['means']['AoA, deg'],marker='s',color='#222222',
                   label='Earlier sustained runs (near sea level)' if i==0 else None)
    ax.set(xlabel='TAS (km/h)',ylabel='Body AoA (degrees)',title='Full pull: achieved AoA versus permitted boundary')
    ax.legend(fontsize=8);ax.invert_xaxis()
    axes[0,1].plot(t,alpha,color='#25699f');axes[0,1].axvspan(8,t[-1],color='#25699f',alpha=.08)
    axes[0,1].set(xlabel='Time (s)',ylabel='AoA (degrees)',title='AoA stabilizes while speed continues falling')
    axes[1,0].plot(t,speed,color='#25699f');axes[1,0].set(xlabel='Time (s)',ylabel='TAS (km/h)',title='Recording ends above sustained-turn speed')
    ax=axes[1,1];ax.plot(t,d['Altitude, m'],color='#666666',label='Altitude')
    ax.set(xlabel='Time (s)',ylabel='Altitude (m)',title='Altitude and measured body-axis load vary')
    right=ax.twinx();right.plot(t,d['Load factor, g'],color='#b47a22');right.set_ylabel('Body-axis load (g)',color='#b47a22')
    for ax in axes.flat:ax.grid(alpha=.2)
    fig.suptitle('Yak-3 France · held-pitch deceleration · recorded mass 2,490.4 kg',fontsize=14)
    fig.text(.03,.018,'Model dots are fully balanced static states at nearby recorded TAS/altitude and matched total mass; they are not a replay of the transient.\nThe early pull is already underway at t=0. The shaded AoA plateau does not imply constant energy or zero vertical acceleration.',fontsize=9)
    fig.tight_layout(rect=(0,.065,1,.96));fig.savefig(OUT/'aoa-versus-speed.png',dpi=170);fig.savefig(OUT/'aoa-versus-speed.svg')


if __name__=='__main__':main()
