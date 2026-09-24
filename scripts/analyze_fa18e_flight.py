"""Compare heading-derived F/A-18E max-pull telemetry with production EM equations."""
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import numpy as np
from scipy.optimize import brentq
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from analyze_prop_flights import read, brief
from em_solver import settings
from em_sampling import sample_boundary_column, sample_column, worker_solver

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'analysis/fa18e-flight-comparison'
FILE=ROOT/'FlightTestData/fa_18e_maxinstructorpullfromhighspeed.csv'
NAME='fa_18e_block_2'
G=9.80665
FUEL=(16672.-14620.)/6667.*100.


def derivative(t,y,centers,width=1.):
    # Fit against actual irregular timestamps, not a presumed sampling rate.
    # Polynomial derivatives reduce heading quantization without crossing 360°.
    result=[]
    for center in centers:
        mask=abs(t-center)<=width/2
        z=t[mask]-center
        result.append(np.polynomial.polynomial.polyfit(z,y[mask],2)[1])
    return np.array(result)


def config(altitude,fuel=FUEL):
    return settings(dict(aircraft=[NAME],instructor=True,fuel_percent=fuel,
        altitude_m=float(altitude),flaps_percent=0.,throttle=1.1,afterburner=True))


def column(speed,altitude,fuel=FUEL,full=False):
    cfg=config(altitude,fuel)
    task=(NAME,json.dumps(cfg),float(speed),None)
    return (sample_column if full else sample_boundary_column)(task),cfg


def main():
    OUT.mkdir(exist_ok=True)
    data=read(FILE);t=data['Time, s'];assert np.all(np.diff(t)>0)
    heading=np.rad2deg(np.unwrap(np.deg2rad(data['Compass'])))
    # Diagnostic only: for nearly level, nearly zero-sideslip flight, the
    # horizontal nose/velocity offset is asin(sin(alpha)*sin(Euler roll)).
    # This run turns left (decreasing Compass, positive logged roll), so
    # increasing offset adds to nose rate but not to flight-path curvature.
    offset=np.rad2deg(np.arcsin(np.sin(np.deg2rad(data['AoA, deg']))*
                               np.sin(np.deg2rad(data['Roll angle, deg']))))
    speed=data['TAS, km/h'];energy=data['Altitude, m']+(speed/3.6)**2/(2*G)
    centers=np.arange(.5,t[-1]-.5,.25)
    rate=-derivative(t,heading,centers)
    trace={k:np.interp(centers,t,v) for k,v in data.items()}
    trace.update(time_s=centers,heading_rate_dps=rate,
        heading_rate_half_second_dps=-derivative(t,heading,centers,.5),
        heading_rate_two_second_dps=-derivative(t,heading,centers,2.),
        ps_mps=derivative(t,energy,centers),
        alpha_rate_dps=derivative(t,data['AoA, deg'],centers),
        climb_mps=derivative(t,data['Altitude, m'],centers))
    trace['estimated_flightpath_rate_dps']=rate-derivative(t,offset,centers)
    with (OUT/'derived-telemetry.csv').open('w') as f:
        w=csv.writer(f);w.writerow(trace);w.writerows(zip(*trace.values()))
    steady=t>=60
    steady_rate=-(heading[-1]-np.interp(60,t,heading))/(t[-1]-60)
    mean=lambda k:float(np.trapz(data[k][steady],t[steady])/(t[steady][-1]-t[steady][0]))
    summary=dict(file=str(FILE.relative_to(ROOT)),sha256=hashlib.sha256(FILE.read_bytes()).hexdigest(),
        samples=len(t),duration_s=float(t[-1]),fuel_percent_mass_equivalent=FUEL,
        condition='User confirmed clean, flaps retracted, full afterburner, fully upgraded, default ammunition/countermeasures.',
        mass_note='Recorded mass 16672 kg; clean 30% fuel 16620.10 kg. Primary comparison matches total mass using equivalent fuel (30.77846%); ammunition location/inertia are not known. Nominal 30% sensitivity is separately reported.',
        derivative='Unwrap Compass then differentiate a quadratic local fit over 1 s of actual timestamps. Signed heading decreases; plot absolute rate. Also calculate 0.5 and 2 s windows.',
        late=dict(start_s=60,end_s=float(t[-1]),heading_rate_dps=float(steady_rate),
            means={k:mean(k) for k in data},
            ps_endpoint_mps=float((energy[-1]-np.interp(60,t,energy))/(t[-1]-60))),
        peak_heading_rate=dict(time_s=float(centers[rate.argmax()]),rate_dps=float(rate.max()),
            tas_kmh=float(trace['TAS, km/h'][rate.argmax()])))
    (OUT/'telemetry.json').write_text(json.dumps(summary,indent=2)+'\n')
    records=[];started=time.monotonic()
    for i in range(0,len(centers),4):
        speed=float(trace['TAS, km/h'][i]);alt=float(trace['Altitude, m'][i])
        col,cfg=column(speed,alt);edge=col['boundary'];solver=worker_solver(NAME,json.dumps(cfg))
        observed=float(rate[i]);load=math.hypot(1.,math.radians(observed)*speed/3.6/G)
        matched=solver.solve(speed,load,edge['solution'] if edge else None,exhaustive=False)
        records.append(dict(time_s=float(centers[i]),observed={k:float(v[i]) for k,v in trace.items()},
            boundary=brief(edge),boundary_status=col['boundary_status'],mass_kg=col['mass']['mass'],
            matched_heading_rate_point=brief(matched)))
        if len(records)%10==0:print('time samples',len(records),'seconds',round(time.monotonic()-started,1),flush=True)
    (OUT/'time-comparison.json').write_text(json.dumps(records,indent=2)+'\n')
    altitude=summary['late']['means']['Altitude, m'];late_speed=summary['late']['means']['TAS, km/h']
    latecol,cfg=column(late_speed,altitude,full=True)
    result=dict(telemetry=summary,late_boundary=brief(latecol['boundary']),
        late_sustained=[brief(p) for p in latecol['sustained']],sensitivity=[])
    peak=int(rate.argmax())
    peakcol,_=column(trace['TAS, km/h'][peak],trace['Altitude, m'][peak])
    result['peak_comparison']=dict(observed={k:float(v[peak]) for k,v in trace.items()},
                                  boundary=brief(peakcol['boundary']))
    def ps(speed):return column(speed,altitude)[0]['boundary']['ps_mps']
    root=brentq(ps,400.,480.,xtol=.01)
    result['max_pull_ps_zero']=brief(column(root,altitude)[0]['boundary'])
    for speed in [450.,800.,1200.]:
        a,_=column(speed,altitude,30.);b,_=column(speed,altitude,FUEL)
        result['sensitivity'].append(dict(speed_kmh=speed,nominal_30_percent=brief(a['boundary']),
                                         matched_mass=brief(b['boundary'])))
    (OUT/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    fig,ax=plt.subplots(3,1,figsize=(11,12),sharex=True)
    x=trace['TAS, km/h'];mx=[r['observed']['TAS, km/h'] for r in records]
    ax[0].plot(x,rate,label='Flight: |d(unwrapped heading)/dt|, 1 s fit',color='#146fa1')
    ax[0].plot(x,trace['heading_rate_half_second_dps'],alpha=.25,lw=.8,color='#146fa1',label='Flight: 0.5 s fit')
    ax[0].plot(x,trace['estimated_flightpath_rate_dps'],color='#6b8791',ls='--',lw=1,
               label='Flight-path estimate: remove changing nose/AoA offset (diagnostic)')
    ax[0].plot(mx,[r['boundary']['turn_dps'] for r in records],color='#db6d26',label='Model: Instructor maximum-pull boundary')
    for p in latecol['sustained']:ax[0].scatter([late_speed],[p['turn_dps']],color='green',marker='x',s=80,label='Model: Ps = 0 at late-flight speed')
    p=result['max_pull_ps_zero'];ax[0].scatter([p['speed_kmh']],[p['turn_dps']],color='#db6d26',s=45,label='Model: max-pull Ps = 0')
    ax[0].set_ylabel('Turn rate (deg/s)');ax[0].legend(fontsize=8)
    ax[1].plot(x,trace['AoA, deg'],label='Flight body AoA',color='#146fa1')
    ax[1].plot(mx,[r['boundary']['alpha_deg'] for r in records],label='Model boundary body AoA',color='#db6d26')
    ax[1].set_ylabel('Body AoA (deg)');ax[1].legend(fontsize=8)
    ax[2].plot(x,trace['ps_mps'],label='Flight: d(h + V²/2g)/dt',color='#146fa1')
    ax[2].plot(mx,[r['boundary']['ps_mps'] for r in records],label='Model Ps at maximum-pull boundary',color='#db6d26')
    ax[2].axhline(0,color='gray',lw=.7);ax[2].set_ylabel('Specific excess power (m/s)');ax[2].legend(fontsize=8)
    for a in ax:a.grid(alpha=.2)
    ax[2].set_xlabel('True airspeed (km/h)')
    fig.suptitle('F/A-18E maximum Instructor pull: flight vs current static EM model\nMass 16,672 kg; model altitude matched at each sample; full afterburner, flaps retracted')
    fig.tight_layout();fig.savefig(OUT/'comparison.png',dpi=170);fig.savefig(OUT/'comparison.svg');plt.close(fig)
    print('late',summary['late']['heading_rate_dps'],late_speed,'sustained',[(p['turn_dps'],p['alpha_deg']) for p in latecol['sustained']],flush=True)
    print('maxpull zero',result['max_pull_ps_zero']['speed_kmh'],result['max_pull_ps_zero']['turn_dps'],flush=True)

if __name__=='__main__':main()
