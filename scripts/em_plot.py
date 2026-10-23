import csv
import io
import json
import orjson
import math
import time
from bisect import bisect_left
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np
import contourpy
from scipy.interpolate import PchipInterpolator
from em_stationary_baseline import presentation

LEVELS=[-400.,-200.,-100.,0.,100.]
SYMBOL_FONT=Path(__file__).resolve().parents[1]/'app/fonts/wt-symbols.ttf'
font_manager.fontManager.addfont(SYMBOL_FONT)
SYMBOL_FAMILY=font_manager.FontProperties(fname=SYMBOL_FONT).get_name()
SPEED_BOUNDARY_LABELS={
    'VNE speed boundary':('VNE',''),
    'Mach limit speed boundary':('VNE','Mach limit'),
    'Flap IAS limit speed boundary':('Flap speed limit','Structure'),
    'Flap automatic IAS limit speed boundary':('Flap speed limit','Automatic flap controls'),
    'Flap automatic Mach limit speed boundary':('Flap speed limit','Automatic flap controls'),
}


def speed_boundary_labels(aircraft, data=None):
    edges={}
    for point in aircraft['boundary']:
        kind=point.get('edge_kind')
        if (point.get('vertical_edge') and kind in SPEED_BOUNDARY_LABELS
                and point.get('turn_dps') is not None):
            edges.setdefault((point['speed_kmh'],kind),[]).append(point['turn_dps'])
    labels=[dict(speed_kmh=speed,turn_dps=(min(rates)+max(rates))/2,
                 text='\n'.join(filter(None,SPEED_BOUNDARY_LABELS[kind])))
            for (speed,kind),rates in edges.items() if max(rates)>min(rates)]
    redline=aircraft.get('speed_limit') or {}
    speed=redline.get('deployment_speed_kmh')
    if (data is not None and speed is not None
            and data['settings']['speed_min_kmh']<=speed<=data['settings']['speed_max_kmh']
            and (not redline.get('enforced') or speed<redline['speed_kmh']-.01)):
        description='Higher speeds require prior deployment; already-deployed flaps remain usable up to the envelope limit.'
        labels.append(dict(speed_kmh=speed,turn_dps=data['plot_max_turn']/2,
            text='Flap deployment limit',dashed=True,
            description=description))
    for crossing in (aircraft.get('over_rev') or {}).get('crossings',[]):
        vne=redline.get('vne_tas_kmh',(aircraft.get('over_rev') or {}).get('vne_tas_kmh',math.inf))
        if crossing['speed_kmh']>=vne:continue
        if data is not None and not data['settings'].get('speed_min_kmh',0.)<=crossing['speed_kmh']<=data['settings'].get('speed_max_kmh',math.inf):continue
        labels.append(dict(speed_kmh=crossing['speed_kmh'],turn_dps=(data or {}).get('plot_max_turn',20.)/2,
            text='Over-rev boundary',dashed=True,
            description=f"Engine {crossing['engine_index']+1}: {crossing['rpm_limit']:.0f} RPM; over-rev on {crossing['over_rev_side']} side. Steady axial-flow estimate; turning-flight RPM can differ."))
    return labels


def nullable_grid(values):
    array=np.asarray(values);out=array.astype(object)
    out[~np.isfinite(array)]=None
    return out.tolist()


def display_discontinuities(config,aircraft,xs,turn,loads,z):
    """Draw narrow formula switches at one speed with separate one-sided limits.

    Only presentation coordinates move, by less than the horizontal chart
    tolerance. A zero-width masked column keeps contours from interpolating
    across the jump. Physical samples and unresolved numerical gaps stay intact.
    """
    from em_accuracy import speed_tolerance
    tolerance=speed_tolerance(config)
    interpolation=aircraft.get('interpolation',{})
    columns={c['speed_kmh']:c for c in aircraft.get('columns',())}
    displayed=[]
    for event in interpolation.get('native_discontinuities',()):
        lo,hi=event['speed_interval_kmh']
        if not 0.<hi-lo<=tolerance:continue
        if any(a<hi and b>lo for a,b in interpolation.get('unresolved_speed_intervals',())):continue
        sides=[columns.get(v) for v in (lo,hi)]
        if not all(c and c.get('boundary_status')=='verified limit' and
                   (c.get('boundary') or {}).get('valid') for c in sides):continue
        li,ri=np.searchsorted(xs,[lo,hi])
        if ri>=len(xs) or xs[li]!=lo or xs[ri]!=hi:continue
        if not all(np.isfinite(z[:,i]).any() for i in (li,ri)):continue
        outline=aircraft['boundary']
        left=next((p for p in outline if p['speed_kmh']==lo and p['turn_dps'] is not None),None)
        right=next((p for p in outline if p['speed_kmh']==hi and p['turn_dps'] is not None),None)
        if left is None or right is None:continue
        center=(lo+hi)*.5
        indices=np.r_[np.arange(li+1),li,li,ri,np.arange(ri,len(xs))]
        xs=xs[indices].copy();xs[li+1:li+4]=center
        turn=turn[:,indices];loads=loads[:,indices];z=z[:,indices].copy()
        z[:,li+2]=np.nan
        limits=[dict(left,speed_kmh=center,sample_speed_kmh=lo,mach_discontinuity_side='left'),
                dict(speed_kmh=center,turn_dps=None,at_plot_ceiling=False,edge_kind='Native Mach discontinuity'),
                dict(right,speed_kmh=center,sample_speed_kmh=hi,mach_discontinuity_side='right')]
        aircraft['boundary']=sorted([p for p in outline if not lo<p['speed_kmh']<hi]+limits,
                                    key=lambda p:p['speed_kmh'])
        displayed.append(dict(speed_kmh=center,sampled_speed_interval_kmh=[lo,hi],
            method='Separate one-sided limits at chart horizontal resolution; no cross-jump interpolation'))
    aircraft['display_discontinuities']=displayed
    return xs,turn,loads,z


def contour_levels(aircraft,data=None):
    return sorted((data or {}).get('settings',{}).get('sep_contour_levels_mps',LEVELS))


@presentation
def contour_paths(data,levels=None,*,prepared_matrices=None,march_drawn=False):
    if data.get('stationary_full_chart'):
        from em_stationary_surface import contours as stationary
        return stationary(data, levels)
    output={}
    for aircraft in data['aircraft']:
        if aircraft.get('chart_solver')=='march':
            if march_drawn and levels is None and 'contours' in aircraft:
                # S7: enrich has just drawn these (em_march_chart.enrich, the same calculated levels)
                output[aircraft['id']]=aircraft['contours'];continue
            from em_march_chart import contours as march_contours
            output[aircraft['id']]=march_contours(aircraft,levels if levels is not None else contour_levels(aircraft,data))
            continue
        prepared=(prepared_matrices or {}).get(aircraft['id'])
        x,y,z=prepared if prepared is not None else matrices(data,aircraft)
        paths=[]
        if z.count()>3 and np.ma.max(z)>np.ma.min(z):
            contour=contourpy.contour_generator(x=x,y=y,z=z,name='mpl2014',corner_mask=False)
            for level in levels if levels is not None else contour_levels(aircraft,data):
                for segment in contour.lines(level)[0]:
                    if len(segment)>1:paths.append(dict(level=float(level),x=segment[:,0].tolist(),y=segment[:,1].tolist()))
        output[aircraft['id']]=paths
    return output


@presentation
def matrices(data, aircraft):
    from em_continuous_boundary import mask_surface
    if 'surface' in aircraft:
        s=aircraft['surface'];y=np.array(s['turn_dps'])
        x=np.broadcast_to(np.asarray(s['x']),y.shape)
        z=np.ma.masked_invalid(np.array(s['z'],dtype=float))
        return x,y,mask_surface(aircraft,x,y,z)
    nx=len(data['speeds_kmh']); ny=len(data['loads_g'])
    points=aircraft['points']
    x=np.array([p['speed_kmh'] for p in points]).reshape(ny,nx)
    y=np.array([p['turn_dps'] for p in points]).reshape(ny,nx)
    z=np.ma.array(np.array([p['ps_mps'] for p in points],dtype=float).reshape(ny,nx),
                  mask=np.array([not p['valid'] for p in points]).reshape(ny,nx))
    return x,y,mask_surface(aircraft,x,y,z)


@presentation
def smooth_surface(data,aircraft,*,surface_format='lists'):
    from em_sampling import speed_interpolate
    from em_load_limits import LOWER_SPEED_KMH,LOWER_LOAD_G


    columns=aircraft['columns'];nx=2*data['settings']['surface_resolution']-1;nf=1201
    outline=aircraft.get('boundary_columns',columns)
    lower_speed_cap=any((c.get('load_search_limit') or {}).get('at_or_below_speed_kmh')==LOWER_SPEED_KMH for c in columns+outline)
    if data.get('preview'):nx=min(nx,241);nf=121
    xs=np.linspace(data['settings']['speed_min_kmh'],data['settings']['speed_max_kmh'],nx)


    xs=np.unique(np.concatenate([xs,[c['speed_kmh'] for c in columns],[c['speed_kmh'] for c in outline]]));nx=len(xs)


    interpolation=aircraft.get('interpolation',{})
    uncertain=interpolation.get('unresolved_speed_intervals',[])+interpolation.get('mach_transition_intervals',[])
    discontinuities=[r['speed_interval_kmh'] for r in interpolation.get('native_discontinuities',[])]
    uncertain=uncertain+discontinuities
    separators=[]
    for lo,hi in uncertain:
        cuts=[lo,*sorted(c['speed_kmh'] for c in columns if lo<c['speed_kmh']<hi),hi]
        separators.extend(a+(b-a)*.5 for a,b in zip(cuts,cuts[1:]) if a<b)
    if separators:xs=np.unique(np.concatenate([xs,separators]));nx=len(xs)


    edges=[edge+delta for span in aircraft.get('sweep_excluded_speeds_kmh',[]) for edge in span for delta in [-.01,.01]
           if data['settings']['speed_min_kmh']<edge+delta<data['settings']['speed_max_kmh']]
    if edges:xs=np.unique(np.concatenate([xs,edges]));nx=len(xs)
    z=np.full((nf,nx),np.nan);turn=np.zeros((nf,nx));caps=np.full(nx,np.nan)
    boundary=[];run=[]
    def finish(group):
        if len(group)<2:return
        speeds=[c['speed_kmh'] for c in group]
        mask=(xs>=speeds[0])&(xs<=speeds[-1]);x=xs[mask]
        cap=PchipInterpolator(speeds,[c['boundary']['load_g'] for c in group])(x)
        caps[mask]=cap
    for column in outline+[None]:
        if column is not None and column.get('boundary') and column['boundary_status'] in ('verified limit','plot ceiling'):run.append(column)
        else:finish(run);run=[]


    from em_surface import envelope_limits,mesh_limits
    limits=envelope_limits(columns,xs)
    fitted=np.isfinite(limits).all(axis=1)
    caps[fitted]=limits[fitted,1]


    sampled_limits=mesh_limits(columns,xs)
    sampled=np.isfinite(sampled_limits).all(axis=1)
    lower=np.where(fitted,limits[:,0],np.where(sampled,sampled_limits[:,0],1.))
    upper=np.where(np.isfinite(caps),caps,np.where(sampled,sampled_limits[:,1],lower))
    global_cap=data['settings'].get('global_load_cap_g')
    aircraft_cap=None
    from em_aircraft_region import REVISION as region_revision
    has_region=any((c.get('load_search_limit') or {}).get('revision') in
                   (region_revision,'preliminary-forces-v1') for c in outline)
    if data['settings'].get('aircraft_search_region',False) and has_region and len(outline)>=2:
        # Include columns governed by the outer cap or an unavailable estimate;
        # holding the last tight estimate there could clip a valid boundary.
        aircraft_cap=np.interp(xs,[c['speed_kmh'] for c in outline],
            [(c.get('load_search_limit') or {}).get('load_g',global_cap or 64.) for c in outline])
        caps=np.minimum(caps,aircraft_cap)
        upper=np.minimum(upper,aircraft_cap)
    reference_cap=None
    if data['settings'].get('reference_load_cap',False):
        from em_reference_envelope import reference_ceiling
        reference_cap=np.array([reference_ceiling(float(x)) for x in xs])
        caps=np.minimum(caps,reference_cap)
        upper=np.minimum(upper,reference_cap)
    if global_cap is not None:
        caps=np.minimum(caps,global_cap)
        upper=np.minimum(upper,global_cap)
    legacy_low_cap=data['settings'].get('low_speed_load_cap',False) and reference_cap is None
    if legacy_low_cap:
        below=xs<300.
        caps[below]=np.minimum(caps[below],8.)
        upper[below]=np.minimum(upper[below],8.)
        if lower_speed_cap:
            below=xs<=LOWER_SPEED_KMH
            caps[below]=np.minimum(caps[below],LOWER_LOAD_G)
            upper[below]=np.minimum(upper[below],LOWER_LOAD_G)
    fraction=np.linspace(0.,1.,nf)
    lower_turn=np.sqrt(np.maximum(0.,lower*lower-1.))
    upper_turn=np.sqrt(np.maximum(0.,upper*upper-1.))
    loads=np.sqrt(1.+(lower_turn[None,:]+fraction[:,None]*(upper_turn-lower_turn)[None,:])**2)
    loads[0]=lower;loads[-1]=upper
    z=speed_interpolate(columns,xs,loads)


    solved_speeds=np.isin(xs,[c['speed_kmh'] for c in columns])
    for lo,hi in uncertain:


        z[:,(xs>lo)&(xs<hi)&~solved_speeds]=np.nan


    from em_speed_seam import plot_values
    for certificate in aircraft.get('interpolation',{}).get('certified_speed_interiors',[]):
        lo,hi=certificate['speed_interval_kmh'];mask=(xs>lo)&(xs<hi)&~solved_speeds
        z[:,mask]=plot_values(columns,certificate,xs[mask],loads[:,mask])
    # Known discontinuities cannot be restored by exact samples or old seam
    # certificates. A NaN column splits every contour, including Ps=0.
    for lo,hi in discontinuities:z[:,(xs>lo)&(xs<hi)]=np.nan
    turn=np.degrees(9.8100004196167*np.sqrt(loads**2-1.)/(xs[None,:]/3.6))
    for i,x in enumerate(xs):
        cap=caps[i]
        index=max(0,min(len(outline)-2,int(np.searchsorted([c['speed_kmh'] for c in outline],x))-1))
        verified=all(c['boundary_status'] in ('verified limit','plot ceiling') for c in outline[index:index+2])
        if not solved_speeds[i] and any(lo<x<hi for lo,hi in uncertain):verified=False
        rate=math.degrees(9.8100004196167*math.sqrt(max(0.,cap*cap-1.))/(x/3.6)) if np.isfinite(cap) else None
        boundary.append(dict(speed_kmh=float(x),turn_dps=rate if verified else None,
                             at_plot_ceiling=bool(np.isfinite(cap) and (
                                 reference_cap is not None and abs(cap-reference_cap[i])<1e-5 or
                                 aircraft_cap is not None and abs(cap-aircraft_cap[i])<1e-5 or
                                 global_cap is not None and abs(cap-global_cap)<1e-5 or
                                 data['settings']['max_load_g'] is not None and abs(cap-data['settings']['max_load_g'])<1e-5 or
                                 legacy_low_cap and x<300. and abs(cap-8.)<1e-5 or
                                 legacy_low_cap and lower_speed_cap and x<=LOWER_SPEED_KMH and abs(cap-LOWER_LOAD_G)<1e-5 or
                                 any(c['boundary_status']=='plot ceiling' for c in outline[index:index+2])))))


    exact={c['speed_kmh']:c for c in outline}
    for point in boundary:
        column=exact.get(point['speed_kmh'])
        if column and column['boundary_status'] in ('verified limit','plot ceiling'):
            point['turn_dps']=column['boundary']['turn_dps']
            if column['boundary_status']=='plot ceiling':point['at_plot_ceiling']=True
            if column.get('load_search_limit'):point['load_search_limit']=column['load_search_limit']
    edge=aircraft.get('low_speed_edge')
    if edge:
        at=next((i for i,p in enumerate(boundary) if p['speed_kmh']==edge['speed_kmh'] and p['turn_dps'] is not None),None)
        if at is not None:


            boundary.insert(at,dict(speed_kmh=edge['speed_kmh'],turn_dps=0.,at_plot_ceiling=False,
                                    edge_kind=edge['kind'],vertical_edge=True))
            boundary[at+1].update(edge_kind=edge['kind'],vertical_edge=True)
    redline=aircraft.get('speed_limit')
    if redline and redline['enforced']:
        sample=redline['sample_speed_kmh'];column=exact.get(sample)
        if (column and column['boundary_status']=='verified limit' and
            data['settings']['speed_min_kmh']<=redline['speed_kmh']<=data['settings']['speed_max_kmh']):
            at=next((i for i,p in enumerate(boundary) if p['speed_kmh']==sample and p['turn_dps'] is not None),None)
            if at is not None:
                common=dict(speed_kmh=redline['speed_kmh'],sample_speed_kmh=sample,
                    at_plot_ceiling=False,vertical_edge=True,edge_kind=redline['kind']+' speed boundary')
                boundary[at+1:at+1]=[dict(common,turn_dps=column['boundary']['turn_dps']),dict(common,turn_dps=0.)]
    from em_boundary_seam import apply_outline
    aircraft['boundary']=apply_outline(boundary,aircraft.get('interpolation',{}).get('certified_boundary_intervals',[]))
    for lo,hi in discontinuities:
        for point in aircraft['boundary']:
            if lo<point['speed_kmh']<hi:
                point.update(turn_dps=None,edge_kind='Native Mach discontinuity')
    aircraft['numerical_boundaries']=[dict(speed_kmh=c['speed_kmh'],turn_dps=c['boundary']['turn_dps'],load_g=c['boundary']['load_g'])
        for c in outline if c['boundary'] and c['boundary_status']=='unresolved numerical boundary']
    aircraft['numerical_gaps']=[dict(speed_kmh=c['speed_kmh'],**gap,
        turn_dps=math.degrees(9.8100004196167*math.sqrt(gap['load_g']**2-1.)/(c['speed_kmh']/3.6)))
        for c in columns for gap in c.get('numerical_gap_brackets',[])]
    xs,turn,loads,z=display_discontinuities(data['settings'],aircraft,xs,turn,loads,z)
    surface=dict(x=xs,fraction=fraction,load_g=loads,
        mesh='verified boundary and uniform turn fraction',turn_dps=turn,z=z)
    if surface_format=='lists':
        surface={k:(nullable_grid(v) if k=='z' else v.tolist())
                 if isinstance(v,np.ndarray) else v for k,v in surface.items()}
    elif surface_format!='arrays':raise ValueError('Unknown surface format')
    aircraft['surface']=surface
    for column in columns:
        for key in ('_curve','_alpha_curve','_angle_map'):column.pop(key,None)
    # Feed contours from the arrays already in memory, avoiding a round trip
    # through millions of boxed JSON values. Public/exported surface stays lists.
    from em_continuous_boundary import mask_surface
    x=np.broadcast_to(xs,turn.shape)
    return x,turn,mask_surface(aircraft,x,turn,np.ma.masked_invalid(z))


def plot_turn_ceiling(aircraft):
    def edge_points(row):
        return (row.get('boundary') or
                [c['boundary'] for c in row.get('columns',[]) if c.get('boundary')] or
                [p for p in row['points'] if p['valid']])
    max_turn=max((p['turn_dps'] for row in aircraft for p in edge_points(row)
                  if p.get('turn_dps') is not None and math.isfinite(p['turn_dps'])),default=0.)
    return max(10.,float(math.ceil(max_turn*1.05+1.)))


@presentation
def enrich(data,*,surface_format='lists'):
    if data.get('stationary_full_chart'):
        from em_stationary_surface import enrich as stationary
        return stationary(data)
    data['plot_max_load_g']=max(1.1,max((p['load_g'] for a in data['aircraft'] for p in a['points'] if p['valid']),default=1.1))
    prepared={}
    for aircraft in data['aircraft']:
        if aircraft.get('chart_solver')=='march':
            # Alpha-march chart: outline from the verified columns, contours from
            # the solved SEP curves; there is no interpolated SEP surface.
            from em_march_chart import enrich as march_enrich
            march_enrich(data,aircraft)
        elif 'columns' in aircraft:prepared[aircraft['id']]=smooth_surface(data,aircraft,surface_format=surface_format)
        else:prepared[aircraft['id']]=matrices(data,aircraft)


    data['plot_max_turn']=plot_turn_ceiling(data['aircraft'])
    paths_by_aircraft=contour_paths(data,prepared_matrices=prepared,march_drawn=True)
    for aircraft in data['aircraft']:
        paths=paths_by_aircraft[aircraft['id']]
        if aircraft.get('chart_solver')=='march':
            aircraft['contours']=paths;continue
        boundary=[]
        x,y,z=prepared[aircraft['id']]
        if 'surface' not in aircraft:
            mask=np.ma.getmaskarray(z)
            for i in range(x.shape[1]):
                valid=np.where(~mask[:,i])[0]
                if len(valid):
                    j=valid[-1]
                    root_turn=max((p['turn_dps'] for p in aircraft.get('sustained',[]) if p['speed_kmh']==x[j,i]),default=0.)
                    boundary.append(dict(speed_kmh=float(x[j,i]),turn_dps=float(y[j,i]),
                                         at_plot_ceiling=bool(j==x.shape[0]-1)))
                    boundary[-1]['turn_dps']=max(boundary[-1]['turn_dps'],root_turn)
                else:boundary.append(dict(speed_kmh=float(x[0,i]),turn_dps=None,at_plot_ceiling=False))
        aircraft['contours']=paths
        if 'surface' in aircraft:


            zeros=[p for p in paths if p['level']==0.]
            aircraft['sustained_curve']=dict(
                x=[x for p in zeros for x in [*p['x'],None]],
                y=[y for p in zeros for y in [*p['y'],None]])
        if aircraft.get('continuous_pull_boundary'):
            aircraft['boundary']=aircraft['continuous_pull_boundary']
        elif 'surface' not in aircraft:aircraft['boundary']=boundary
    return data


@presentation
def preview_payload(data):
    # Contours consume arrays directly; the preview does not export its surface.
    enrich(data,surface_format='arrays')
    chart={k:v for k,v in data.items() if k!='aircraft'}
    chart['aircraft']=[{k:v for k,v in a.items() if k not in ('columns','surface','boundary_columns','curve_knots','curve_components','curves')}
                       for a in data['aircraft']]
    add_boundary_hover(chart,data)
    return chart


def export_csv(data):
    condition_fields=['torque_gyro','engine_control_mode','aircraft_trim_mode','turn_response_mode','instructor_authority_mode','trim_solver_mode','altitude_m','fuel_percent','throttle','afterburner','extra_mass_kg','default_ammunition','ammunition_vehicle','loadout','stores_released','structural_limits','timestep_hz']
    out=io.StringIO();fields=['aircraft','aircraft_id','aircraft_name','kind','calculation_status','partial_result']+condition_fields+['speed_kmh','load_g','turn_dps','ps_mps','ps_continuous_mps',
          'valid','local_valid','status','chart_usable','chart_status','converged','reasons','alpha_deg','bank_deg','sideslip_deg','sideslip_attitude_deg','ias_kmh','mach','force_error_g',
          'angular_error_rad_s2','history_error','stall_margin_deg','vertical_step_velocity_mps',
          'commands','authority_margin','force_n','moment_nm','engine_force_n','sweep_percent','sweep_available_percent',
          'flaps_requested_percent','flaps_percent','gear_percent','instructor_enabled','instructor','prolonged_pull','propulsion','local_response']
    writer=csv.DictWriter(out,fieldnames=fields);writer.writeheader()
    for aircraft in data['aircraft']:
        march=aircraft.get('chart_solver')=='march'
        for kind,points in [('alpha-march sample' if march else 'grid',aircraft['points']),('Ps=0 refined',aircraft.get('sustained',[])),
                            ('Instructor boundary',aircraft.get('continuous_pull_boundary',[])),
                            ('SEP curve knot',aircraft.get('curve_knots',[]))]:
            for p in points:
                row={key:p.get(key,'') for key in fields}
                row.update(aircraft=aircraft['id'],aircraft_id=aircraft.get('aircraft_id',aircraft['id']),
                           aircraft_name=aircraft.get('name',aircraft['id']),kind=kind,
                           calculation_status=data.get('calculation_status','complete'),partial_result=bool(data.get('calculation_stopped')))
                cfg=aircraft.get('settings',data['settings'])
                if kind=='Instructor boundary':
                    row.update(valid=p['turn_dps'] is not None,instructor_enabled=True,
                               reasons=p.get('edge_kind',''),flaps_percent=cfg['flaps_percent'])
                row.update({k:cfg.get(k,'settled') if k=='turn_response_mode' else cfg.get(k,'nested') if k=='trim_solver_mode' else cfg.get(k,'native') if k=='instructor_authority_mode' else cfg.get(k,'discrete') if k=='aircraft_trim_mode' else cfg.get(k,'optimized') if k=='engine_control_mode' else cfg.get(k,True) if k=='torque_gyro' else cfg.get(k,'') if k=='loadout' else cfg.get(k,False) if k=='stores_released' else cfg[k] for k in condition_fields})
                for key,value in row.items():
                    if isinstance(value,(list,dict)):row[key]=json.dumps(value,separators=(',',':'))
                writer.writerow(row)
    return out.getvalue()


@presentation
def export_figure(data, path, selected=None, levels=None, format=None):
    if data.get('stationary_full_chart'):
        from em_stationary_surface import export_figure as stationary
        return stationary(data, path, selected, levels, format)
    if levels is not None:data=dict(data,settings=dict(data['settings'],sep_contour_levels_mps=list(levels)))
    aircraft=[a for a in data['aircraft'] if selected is None or a['id']==selected]


    with plt.rc_context({'font.family':[SYMBOL_FAMILY,'DejaVu Sans'],
                         'font.size':10,'svg.fonttype':'path'}):
        fig,ax=plt.subplots(figsize=(11,7))
        fig.subplots_adjust(left=.09,right=.97,bottom=.10,top=.87 if len(aircraft)>4 else .90)
        for a in aircraft:
            color={'f_16a_block_15_adf':'#007f92','f_16xl':'#007f92','j6k1':'#bd581c','saab_jas39c':'#bd581c'}.get(a['id'],a['color'])
            if a.get('chart_solver')=='march':
                from em_march_chart import contours as march_contours
                for line in march_contours(a,[n for n in contour_levels(a,data) if n!=0]):
                    ax.plot(line['x'],line['y'],color=color if len(aircraft)>1 else '#526174',lw=1.5,linestyle='dashed')
                    if len(line['x'])>5:
                        i=len(line['x'])//2
                        ax.text(line['x'][i],line['y'][i],f"SEP {line['level']:g}",fontsize=8,ha='center',va='center',
                                bbox=dict(facecolor='white',edgecolor='none',alpha=.75,pad=1))
                x=y=z=None
            else:x,y,z=matrices(data,a)
            if z is not None and z.count()>3 and np.ma.max(z)>np.ma.min(z):
                levels_to_draw=[n for n in contour_levels(a,data) if n!=0]
                if levels_to_draw:
                    contour=ax.contour(x,y,z,levels=levels_to_draw,colors=color if len(aircraft)>1 else '#526174',linewidths=1.5,linestyles='dashed',corner_mask=False)
                    ax.clabel(contour,inline=True,fontsize=8,fmt=lambda v:f'SEP {v:g}')
            boundary=a['boundary']
            ax.plot([p['speed_kmh'] for p in boundary],[np.nan if p['turn_dps'] is None else p['turn_dps'] for p in boundary],
                    '-',color=color,lw=2.8,zorder=3.5,clip_on=False,label=a['name'])
            for label in speed_boundary_labels(a,data):
                cfg=data['settings']
                left=label['speed_kmh']>cfg['speed_min_kmh']+.75*(cfg['speed_max_kmh']-cfg['speed_min_kmh'])
                if label.get('dashed'):
                    ax.axvline(label['speed_kmh'],color=color,lw=1.4,linestyle='--',alpha=.7)
                ax.annotate(label['text'],(label['speed_kmh'],label['turn_dps']),
                            xytext=(-7 if left else 7,0),textcoords='offset points',
                            ha='right' if left else 'left',va='center',rotation=90,color=color,fontsize=9,
                            bbox=dict(facecolor='white',edgecolor='none',alpha=.85,pad=1.5),zorder=5)
            if data.get('show_numerical_diagnostics') and a.get('numerical_boundaries'):
                ax.scatter([p['speed_kmh'] for p in a['numerical_boundaries']],[p['turn_dps'] for p in a['numerical_boundaries']],
                           marker='x',color='#b45a12',s=22,label='_nolegend_')
            if data.get('show_numerical_diagnostics') and a.get('numerical_gaps'):
                ax.scatter([p['speed_kmh'] for p in a['numerical_gaps']],[p['turn_dps'] for p in a['numerical_gaps']],
                           marker='o',facecolors='none',edgecolors='#b45a12',s=16,label='_nolegend_')


            roots={p['speed_kmh']:p for p in a.get('sustained',[])}
            root_curve=a.get('sustained_curve',dict(x=data['speeds_kmh'],y=[roots[v]['turn_dps'] if v in roots else np.nan for v in data['speeds_kmh']]))
            if 0. in contour_levels(a,data):
                ax.plot(root_curve['x'],root_curve['y'],
                        color=color,lw=4,linestyle='--',label='_nolegend_')
                visible=[(x,y) for x,y in zip(root_curve['x'],root_curve['y']) if x is not None and y is not None and np.isfinite(y)]
                if visible:
                    px,py=visible[len(visible)//2]
                    ax.text(px,py,'SEP 0',color=color,fontsize=8,ha='center',va='center',
                            bbox=dict(facecolor='white',edgecolor='none',alpha=.8,pad=.8))
        # Draw the analytic guides independently of the adaptive trim columns.
        v=np.geomspace(data['settings']['speed_min_kmh'],data['settings']['speed_max_kmh'],1201)
        for n in [2,4,6,9,12,16]:
            if n>data['plot_max_load_g']:continue
            rate=np.degrees(9.8100004196167*np.sqrt(n*n-1)/(v/3.6))
            guide,=ax.plot(v,rate,color='#999999',alpha=.2,lw=.6)
            guide.get_path().should_simplify=False
        cfg=data['settings']
        ax.set_aspect('auto')
        ax.set(xlim=(cfg['speed_min_kmh'],cfg['speed_max_kmh']),ylim=(0,data['plot_max_turn']),
               xlabel='True airspeed (km/h)',ylabel='Turn rate (°/s)')
        fig.suptitle('Energy–maneuverability',x=.5,y=.98,fontsize=12)
        ax.grid(alpha=.15)
        fig.legend(loc='upper center',bbox_to_anchor=(.5,.945),ncol=min(4,len(aircraft)),frameon=False,fontsize=9)
        fig.savefig(path,dpi=180,format=format);plt.close(fig)


def boundary_limit_label(state, reason=None):
    limit=state.get('envelope_limit') or {}
    kind=limit.get('kind') or reason
    if kind=='control':
        return {2:'Aileron authority',3:'Elevator authority',4:'Rudder authority'}.get(
            limit.get('axis'),'Control surface authority')
    return {'stall':'Stall', 'Instructor pitch':'AoA limiter (Instructor)',
            'wing force':'Structure (wing load)',
            'engine RPM':'Engine RPM limit',
            'pitch response':'Elevator effectiveness (pitch-response reversal)',
            'trim fold':'Trim equilibrium limit'}.get(kind,'Aircraft limit not identified')


def add_boundary_limits(aircraft, source):
    search='Search limit; aircraft limit not established'
    groups=[];run=[]
    for column in source.get('boundary_columns',source.get('columns',[]))+[None]:
        if column is not None and column.get('boundary') and column['boundary_status'] in ('verified limit','plot ceiling'):
            label=search if column['boundary_status']=='plot ceiling' else boundary_limit_label(
                column['boundary'],column.get('boundary_reason'))
            run.append((column['speed_kmh'],label))
        else:
            if run:groups.append(dict(run))
            run=[]


    measured={p['speed_kmh']:p for c in source.get('interpolation',{}).get('certified_boundary_intervals',[])
              for p in c['points']}
    for group in groups:
        lo,hi=min(group),max(group)
        group.update({v:boundary_limit_label(p) for v,p in measured.items() if lo<=v<=hi})
    groups=[(sorted(g),g) for g in groups]
    edges={'Over-rev speed boundary':'Engine RPM limit (steady axial-flow estimate)',
           'Unresolved RPM limit speed boundary':'RPM search unresolved; calculation stopped',
           'VNE speed boundary':'Structure (VNE speed limit)',
           'Mach limit speed boundary':'Structure (Mach speed limit)',
           'stall-speed edge':'Stall',
           'Instructor pitch minimum-speed edge':'AoA limiter (Instructor)',
           'Instructor minimum-speed edge':'Instructor minimum-speed limit',
           'speed-range edge':'Selected speed range; aircraft limit not established',
           'low-speed feasibility edge':'Low-speed feasibility limit'}
    edges.update({kind:' · '.join(filter(None,label)) for kind,label in SPEED_BOUNDARY_LABELS.items()
                  if kind.startswith('Flap ')})
    # The march chart's high-speed edges (em_march_chart.outline): forward-IAS limits lean, the others stand vertical.
    edges.update({'VNE (forward IAS) speed boundary':'Structure (VNE speed limit on forward IAS)',
                  'Flap IAS limit (forward IAS) speed boundary':'Flap speed limit on forward IAS · Structure',
                  'Flap automatic IAS limit (forward IAS) speed boundary':'Flap speed limit on forward IAS · Automatic flap controls',
                  'Forward IAS limit (forward IAS) speed boundary':'Forward IAS speed limit',
                  'Wing sweep limit':'Wing sweep limit (selected sweep unavailable)',
                  'Wing sweep limit speed boundary':'Wing sweep limit (selected sweep unavailable)',
                  'Flaps retracted at this wing sweep speed boundary':'Flaps retracted at this wing sweep',
                  'Propeller braking edge':'Propeller braking (level flight rejected at higher speed)',
                  'High-speed feasibility edge':'High-speed feasibility limit (level flight rejected at higher speed)'})

    def high_speed_label(point):
        # The level-flight rejection that ends the run here (the envelope trace's 'edge' event), e.g. an engine over-rev.
        reason=next((str(e.get('reason','')) for e in (source.get('envelope_trace') or {}).get('events',[])
                     if e['kind']=='edge' and abs(min(e['speed_kmh'])-point['speed_kmh'])<1e-6),'')
        if 'engine RPM limit' in reason:return 'Engine RPM limit (level flight over-revs at higher speed)'
        if reason.startswith('level flight: '):return 'High-speed feasibility limit ('+reason.removeprefix('level flight: ')+' in level flight at higher speed)'
        return edges['High-speed feasibility edge']
    for point in aircraft['boundary']:
        label='Aircraft limit not identified'
        speed=point.get('sample_speed_kmh',point['speed_kmh'])
        for speeds,labels in groups:
            if not speeds[0]<=speed<=speeds[-1]:continue
            i=bisect_left(speeds,speed)
            if speeds[i]==speed:label=labels[speed]
            else:
                left,right=labels[speeds[i-1]],labels[speeds[i]]
                label=left if left==right else 'Limit transition: '+left+' / '+right
            break
        sample=measured.get(speed)
        if sample and point['turn_dps']==sample['turn_dps']:
            label=boundary_limit_label(sample)
        if source.get('continuous_pull_boundary'):
            label='Prescribed continuous pull; aircraft limit not established'
        if point.get('vertical_edge') or point.get('leaning_edge'):
            edge=point.get('edge_kind')
            label=high_speed_label(point) if edge=='High-speed feasibility edge' else edges.get(edge,label)
            if edge=='Roll-leveling boundary transition':label='Roll-leveling transition · '+label
        if point.get('mach_event'):label='Mach transition · '+label
        if point.get('native_mach_uncertainty'):
            label+=' · Sampled numerical uncertainty; interpolation tolerance not established'
        if point.get('at_plot_ceiling'):label=search
        if point['turn_dps'] is None:label='Unresolved boundary; aircraft limit not established'
        if point.get('edge_kind')=='Native Mach discontinuity':label='Native Mach discontinuity; curve interrupted'
        point['limit_label']=label


def add_boundary_hover(chart, data):
    sources={a['id']:a for a in data['aircraft']}
    for aircraft in chart['aircraft']:
        source=sources[aircraft['id']]
        add_boundary_limits(aircraft,source)
        aircraft['speed_boundary_labels']=speed_boundary_labels(aircraft,chart)
        if source.get('continuous_pull_boundary'):
            for point in aircraft['boundary']:
                point['ps_mps']=None;point['ps_interpolated']=False
            continue
        groups=[];run=[]
        outline=source.get('boundary_columns',source.get('columns',[]))
        for column in outline+[None]:
            if column is not None and column.get('boundary') and column['boundary_status'] in ('verified limit','plot ceiling'):
                run.append(column)
            else:
                if run:groups.append(run)
                run=[]
        for point in aircraft['boundary']:
            point['ps_mps']=None;point['ps_interpolated']=False;point['alpha_deg']=None
            if point.get('vertical_edge') and point['turn_dps']==0.:
                column=next((c for c in outline if c['speed_kmh']==point.get('sample_speed_kmh',point['speed_kmh'])),None)
                level=next((p for p in column['points'] if p['valid'] and p['load_g']==1.),None) if column else None
                if level:
                    point['ps_mps']=level['ps_mps']
                    point['alpha_deg']=level['alpha_deg']
        for group in groups:
            speeds=[c['speed_kmh'] for c in group]
            powers=[c['boundary']['ps_mps'] for c in group]
            angles=[c['boundary']['alpha_deg'] for c in group]
            exact=dict(zip(speeds,powers))
            curve=PchipInterpolator(speeds,powers,extrapolate=False) if len(group)>1 else None
            angle_exact=dict(zip(speeds,angles))
            angle_curve=PchipInterpolator(speeds,angles,extrapolate=False) if len(group)>1 else None
            for point in aircraft['boundary']:
                speed=point.get('sample_speed_kmh',point['speed_kmh'])
                if point.get('vertical_edge') and point['turn_dps']==0.:continue
                # A leaning forward-IAS edge is another limit than the columns' boundary: no AoA or Ps borrowed from it.
                if point.get('leaning_edge'):continue
                if point['turn_dps'] is None or not speeds[0]<=speed<=speeds[-1]:continue
                value=exact.get(speed)
                if value is None and curve is not None:value=float(curve(speed))
                angle=angle_exact.get(speed)
                if angle is None and angle_curve is not None:angle=float(angle_curve(speed))
                if angle is not None and np.isfinite(angle):point['alpha_deg']=float(angle)
                if value is not None and np.isfinite(value):
                    point['ps_mps']=value;point['ps_interpolated']=speed not in exact
                    near=min(group,key=lambda c:abs(c['speed_kmh']-speed))
                    if near.get('boundary_reason')=='trim fold' and not point.get('vertical_edge'):
                        point['edge_kind']='Trim limit · maximum load on the connected equilibrium branch'


        measured={p['speed_kmh']:p for certificate in source.get('interpolation',{}).get('certified_boundary_intervals',[])
                  for p in certificate['points']}
        for point in aircraft['boundary']:
            sample=measured.get(point.get('sample_speed_kmh',point['speed_kmh']))
            if sample and point['turn_dps']==sample['turn_dps']:
                point.update(ps_mps=sample['ps_mps'],alpha_deg=sample['alpha_deg'],ps_interpolated=False)
    return chart


def _public_values(value):
    """Clean only the graph being published; dense arrays are boxed on export."""
    cleaned={}
    def performance_only(value):
        if not isinstance(value,(dict,list)):
            if isinstance(value,(float,np.floating)) and not math.isfinite(value):
                raise ValueError('Nonfinite numerical export')
            return value
        identity=id(value)
        if identity in cleaned:return cleaned[identity]
        if isinstance(value,dict):
            result={k:(surface_export(v) if k=='surface' and isinstance(v,dict)
                       and isinstance(v.get('z'),np.ndarray) else performance_only(v))
                    for k,v in value.items() if k not in ('sticks','trim') and not k.startswith('_')}
        elif not value:result=value
        elif value[0] is None or isinstance(value[0],(int,float,np.number)):
            try:
                try:total=sum(value)
                except TypeError:
                    # Nullable surface rows are already serializable. Validate
                    # their numbers without rebuilding every row and scalar.
                    if any(v is not None and not math.isfinite(v) for v in value):
                        raise ValueError('Nonfinite numerical export')
                else:
                    if not math.isfinite(total) and any(not math.isfinite(v) for v in value):
                        raise ValueError('Nonfinite numerical export')
                result=value
            except TypeError:

                result=[performance_only(v) for v in value]
        else:result=[performance_only(v) for v in value]
        cleaned[identity]=result
        return result
    def surface_export(surface):
        # Validate and box each dense array once, rather than traversing the
        # resulting millions of Python scalars again. Public JSON stays lists
        # with null at masked SEP cells; all other nonfinite values still fail.
        result={}
        for key,value in surface.items():
            if isinstance(value,np.ndarray):
                if key=='z':result[key]=nullable_grid(value)
                else:
                    if not np.isfinite(value).all():raise ValueError('Nonfinite numerical export')
                    result[key]=value.tolist()
            else:result[key]=performance_only(value)
        return result
    return performance_only(value)


def prepare_export_data(data):
    """Prepare downloadable data from an already displayed chart, without solving."""
    return _public_values(data)


@presentation
def prepare_exports(data, *, started_at=None, include_exports=True):
    if 'plot_max_turn' not in data:enrich(data,surface_format='arrays')
    if include_exports:data=prepare_export_data(data)
    points={}


    chart={k:v for k,v in data.items() if k!='aircraft'};chart['aircraft']=[]
    for aircraft in data['aircraft']:
        item={k:v for k,v in aircraft.items() if k not in ('columns','surface','points','sustained','boundary_columns','cells','curve_knots','curve_components','curves')}
        details=aircraft['points']+aircraft.get('sustained',[])
        points[aircraft['id']]=details if include_exports else _public_values(details)
        def brief(point,index):
            return dict({k:point.get(k) for k in ['speed_kmh','load_g','turn_dps','valid','local_valid','status','chart_usable','chart_status','ps_mps','reasons','flaps_percent','flaps_requested_percent']},detail_index=index)
        item['points']=[brief(p,i) for i,p in enumerate(aircraft['points'])]
        item['sustained']=[brief(p,len(aircraft['points'])+i) for i,p in enumerate(aircraft.get('sustained',[]))]
        chart['aircraft'].append(item)
    add_boundary_hover(chart,data);chart['boundary_hover_ready']=True
    if not include_exports:chart=_public_values(chart)
    if started_at is not None:
        chart['calculation_elapsed_s']=chart['elapsed_s']
        chart['elapsed_s']=time.monotonic()-started_at
    return dict(data=data,chart=chart,points=points,exports_ready=include_exports)


def write_exports(data, directory, *, figures=True, started_at=None):
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
    ready=directory/'ready.json';ready.unlink(missing_ok=True)
    prepared=prepare_exports(data,started_at=started_at)
    data=prepared['data'];chart=prepared['chart']
    (directory/'data.json').write_bytes(orjson.dumps(data,option=orjson.OPT_APPEND_NEWLINE|orjson.OPT_SERIALIZE_NUMPY))
    for name,points in prepared['points'].items():
        offsets=[]
        with (directory/(name+'-points.jsonl')).open('wb') as out:
            for point in points:
                offsets.append(out.tell());out.write(orjson.dumps(point,option=orjson.OPT_APPEND_NEWLINE|orjson.OPT_SERIALIZE_NUMPY))
        (directory/(name+'-offsets.json')).write_text(json.dumps(offsets,separators=(',',':')),encoding='utf-8')
    (directory/'chart.json').write_bytes(orjson.dumps(chart,option=orjson.OPT_APPEND_NEWLINE|orjson.OPT_SERIALIZE_NUMPY))
    (directory/'samples.csv').write_text(export_csv(data),encoding='utf-8')
    if figures:
        for extension in ['svg','png','pdf']:export_figure(data,directory/f'diagram.{extension}')
    marker=directory/'ready.json.tmp'
    timing=dict(interactive=True,figures=figures)
    if started_at is not None:timing['total_elapsed_s']=time.monotonic()-started_at
    marker.write_text(json.dumps(timing)+'\n',encoding='utf-8');marker.replace(ready)
