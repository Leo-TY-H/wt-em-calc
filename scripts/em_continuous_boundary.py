"""Attach a prescribed continuous-pull boundary to the ordinary E-M plot.

The curve is a measured offline state trajectory, not an equilibrium Ps
sample. Interior Ps calculations retain their own physical acceptance tests.
Never extrapolate a flight to speeds it did not visit, bridge a rejected
sample, or join successive decelerating branches across a speed reversal.
"""
import math
import numpy as np

CONDITIONS=('altitude_m','fuel_percent','throttle','afterburner','torque_gyro',
            'engine_control_mode','extra_mass_kg','flaps_percent','sweep_percent',
            'structural_limits','timestep_hz')


def trajectory_boundary(report):
    samples=report['samples']
    if len(samples)<2:raise ValueError('A continuous pull needs at least two time samples')
    if not report.get('level_passed'):raise ValueError('Continuous pull failed its level-flight acceptance test')
    previous=None;points=[]
    for row in samples:
        if previous is not None:
            if row['time_s']<=previous['time_s']:raise ValueError('Trajectory time must increase')
            if row['tas_kmh']>=previous['tas_kmh']:
                # The prescribed high-to-low branch ends at its first reversal.
                break
        rate=row['horizontal_turn_dps']
        valid=(rate is not None and math.isfinite(rate) and rate>=0.
               and row.get('physical_step_passed',False)
               and abs(row.get('controller_motion_restore_error',0.))<=1e-8
               and row.get('stall_margin_deg',-1.)>=0.
               and (not report['configuration']['structural_limits']
                    or (min(row.get('wing_load_ratios',[-1.]))>=0.
                        and max(row.get('wing_load_ratios',[math.inf]))<=1.)))
        points.append(dict(speed_kmh=row['tas_kmh'],turn_dps=rate if valid else None,
                           time_s=row['time_s'],alpha_deg=row['alpha_deg'],
                           at_plot_ceiling=False,edge_kind='Continuous full-pitch level pull (experimental)'))
        previous=row
    if sum(p['turn_dps'] is not None for p in points)<2:
        raise ValueError('No accepted continuous-pull segment')
    return list(reversed(points))


def limits(boundary,speeds):
    """Linear display interpolation only, within adjacent accepted samples."""
    speeds=np.asarray(speeds,dtype=float)
    if len(boundary)<2:return np.full(speeds.shape,np.nan)
    knots=np.array([p['speed_kmh'] for p in boundary])
    rates=np.array([p['turn_dps'] for p in boundary],dtype=float)
    if np.any(np.diff(knots)<=0):raise ValueError('Boundary speeds must increase')
    # One binary search per display vertex, rather than allocating a full
    # surface-sized mask for every 48-Hz trajectory sample.
    i=np.clip(np.searchsorted(knots,speeds,side='right')-1,0,len(knots)-2)
    out=rates[i]+(rates[i+1]-rates[i])*(speeds-knots[i])/(knots[i+1]-knots[i])
    out=np.where((speeds>=knots[0])&(speeds<=knots[-1]),out,np.nan)
    exact=np.clip(np.searchsorted(knots,speeds),0,len(knots)-1)
    supported=(np.isfinite(rates[np.maximum(0,exact-1)])&(exact>0)
               |np.isfinite(rates[np.minimum(len(knots)-1,exact+1)])&(exact<len(knots)-1))
    return np.where((speeds==knots[exact])&supported,rates[exact],out)


def install(data,aircraft,report,*,data_commit):
    identity=aircraft.get('aircraft_id',aircraft['id'])
    if identity!=report['aircraft']:raise ValueError('Aircraft differs from trajectory')
    if report.get('data_version')!=data_commit:raise ValueError('Trajectory uses different game data')
    cfg=aircraft.get('settings',data['settings'])
    for key in CONDITIONS:
        if cfg[key]!=report['configuration'][key]:raise ValueError('Trajectory condition differs: '+key)
    boundary=trajectory_boundary(report)
    aircraft['continuous_pull_boundary']=boundary
    aircraft['continuous_pull_method']=dict(
        entry_speed_kmh=report['entry']['speed_kmh'],
        bank_controller=report.get('bank_controller'),
        fixed_fuel=True,full_pitch=True,level_passed=report['level_passed'],
        altitude_error_max_m=max(abs(r['height_error_m']) for r in report['samples']),
        vertical_speed_max_mps=max(abs(r['vertical_speed_mps']) for r in report['samples']),
        exact_game_scheduler=False,global_maximum_validated=False,
        physical_steps_passed=report.get('physical_steps_passed',False),
        scope='Original controller and aircraft kernels; prescribed entry and explicit snapshot clock; fixed fuel/intact.')
    aircraft['instructor_approximation']=dict(kind='continuous full-pitch level pull',experimental=True,
        capability_status='Experimental continuous-pull path; offline aircraft checks passed; live scheduler not validated')
    cfg['instructor']=True
    # Do not present equilibrium samples above the path as accepted interior
    # permission. Nor do transient points become equilibrium Ps samples.
    for collection in ('points','sustained'):
        rows=aircraft.get(collection,[])
        caps=limits(boundary,[r['speed_kmh'] for r in rows])
        for row,cap in zip(rows,caps):
            if not math.isfinite(cap) or row['turn_dps']>cap:
                row['valid']=False
                row['reasons']=list(row.get('reasons',[]))+['outside prescribed continuous pull']
        if collection=='sustained':aircraft[collection]=[r for r in rows if r['valid']]
    aircraft.pop('sustained_curve',None)
    aircraft['valid_points']=sum(bool(p['valid']) for p in aircraft.get('points',[]))
    aircraft['boundary']=boundary
    data.pop('plot_max_turn',None)
    return data


def mask_surface(aircraft,x,y,z):
    boundary=aircraft.get('continuous_pull_boundary')
    if boundary is None:return z
    cap=limits(boundary,np.asarray(x))
    return np.ma.masked_where(~np.isfinite(cap)|(np.asarray(y)>cap),z)
