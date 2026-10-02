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
            and (not redline.get('enforced') or speed<redline['speed_kmh'])):
        labels.append(dict(speed_kmh=speed,turn_dps=data['plot_max_turn']/2,
            text='Flap deployment limit',dashed=True,
            description='Further extension inhibited; higher speeds require prior deployment.'))
    return labels


def nullable_grid(values):
    array=np.asarray(values);out=array.astype(object)
    out[~np.isfinite(array)]=None
    return out.tolist()


def contour_levels(aircraft,data=None):
    return sorted((data or {}).get('settings',{}).get('sep_contour_levels_mps',LEVELS))


def contour_paths(data,levels=None,*,prepared_matrices=None):
    output={}
    for aircraft in data['aircraft']:
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


def smooth_surface(data,aircraft):
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
    aircraft['surface']=dict(x=xs.tolist(),fraction=fraction.tolist(),load_g=loads.tolist(),mesh='verified boundary and uniform turn fraction',turn_dps=turn.tolist(),
                             z=nullable_grid(z))
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


def enrich(data):
    data['plot_max_load_g']=max(1.1,max((p['load_g'] for a in data['aircraft'] for p in a['points'] if p['valid']),default=1.1))
    prepared={}
    for aircraft in data['aircraft']:
        if 'columns' in aircraft:prepared[aircraft['id']]=smooth_surface(data,aircraft)
        else:prepared[aircraft['id']]=matrices(data,aircraft)


    data['plot_max_turn']=plot_turn_ceiling(data['aircraft'])
    paths_by_aircraft=contour_paths(data,prepared_matrices=prepared)
    for aircraft in data['aircraft']:
        paths=paths_by_aircraft[aircraft['id']]
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


def preview_payload(data):
    enrich(data)
    chart={k:v for k,v in data.items() if k!='aircraft'}
    chart['aircraft']=[{k:v for k,v in a.items() if k not in ('columns','surface','boundary_columns')}
                       for a in data['aircraft']]
    add_boundary_hover(chart,data)
    return chart


def export_csv(data):
    condition_fields=['torque_gyro','engine_control_mode','aircraft_trim_mode','turn_response_mode','instructor_authority_mode','trim_solver_mode','altitude_m','fuel_percent','throttle','afterburner','extra_mass_kg','default_ammunition','ammunition_vehicle','structural_limits','timestep_hz']
    out=io.StringIO();fields=['aircraft','aircraft_id','aircraft_name','kind']+condition_fields+['speed_kmh','load_g','turn_dps','ps_mps','ps_continuous_mps',
          'valid','converged','reasons','alpha_deg','bank_deg','sideslip_deg','sideslip_attitude_deg','ias_kmh','mach','force_error_g',
          'angular_error_rad_s2','history_error','stall_margin_deg','vertical_step_velocity_mps',
          'commands','authority_margin','force_n','moment_nm','engine_force_n','sweep_percent','sweep_available_percent',
          'flaps_requested_percent','flaps_percent','gear_percent','instructor_enabled','instructor','prolonged_pull','propulsion','local_response']
    writer=csv.DictWriter(out,fieldnames=fields);writer.writeheader()
    for aircraft in data['aircraft']:
        for kind,points in [('grid',aircraft['points']),('Ps=0 refined',aircraft.get('sustained',[])),
                            ('Instructor boundary',aircraft.get('continuous_pull_boundary',[]))]:
            for p in points:
                row={key:p.get(key,'') for key in fields}
                row.update(aircraft=aircraft['id'],aircraft_id=aircraft.get('aircraft_id',aircraft['id']),
                           aircraft_name=aircraft.get('name',aircraft['id']),kind=kind)
                cfg=aircraft.get('settings',data['settings'])
                if kind=='Instructor boundary':
                    row.update(valid=p['turn_dps'] is not None,instructor_enabled=True,
                               reasons=p.get('edge_kind',''),flaps_percent=cfg['flaps_percent'])
                row.update({k:cfg.get(k,'settled') if k=='turn_response_mode' else cfg.get(k,'nested') if k=='trim_solver_mode' else cfg.get(k,'native') if k=='instructor_authority_mode' else cfg.get(k,'discrete') if k=='aircraft_trim_mode' else cfg.get(k,'optimized') if k=='engine_control_mode' else cfg.get(k,True) if k=='torque_gyro' else cfg[k] for k in condition_fields})
                for key,value in row.items():
                    if isinstance(value,(list,dict)):row[key]=json.dumps(value,separators=(',',':'))
                writer.writerow(row)
    return out.getvalue()


def export_figure(data, path, selected=None, levels=None, format=None):
    if levels is not None:data=dict(data,settings=dict(data['settings'],sep_contour_levels_mps=list(levels)))
    aircraft=[a for a in data['aircraft'] if selected is None or a['id']==selected]


    with plt.rc_context({'font.family':[SYMBOL_FAMILY,'DejaVu Sans'],
                         'font.size':10,'svg.fonttype':'path'}):
        fig,ax=plt.subplots(figsize=(11,7))
        fig.subplots_adjust(left=.09,right=.97,bottom=.10,top=.87 if len(aircraft)>4 else .90)
        for a in aircraft:
            color={'f_16a_block_15_adf':'#007f92','f_16xl':'#007f92','j6k1':'#bd581c','saab_jas39c':'#bd581c'}.get(a['id'],a['color'])
            x,y,z=matrices(data,a)
            if z.count()>3 and np.ma.max(z)>np.ma.min(z):
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
    edges={'VNE speed boundary':'Structure (VNE speed limit)',
           'Mach limit speed boundary':'Structure (Mach speed limit)',
           'stall-speed edge':'Stall',
           'Instructor pitch minimum-speed edge':'AoA limiter (Instructor)',
           'Instructor minimum-speed edge':'Instructor minimum-speed limit',
           'speed-range edge':'Selected speed range; aircraft limit not established',
           'low-speed feasibility edge':'Low-speed feasibility limit'}
    edges.update({kind:' · '.join(filter(None,label)) for kind,label in SPEED_BOUNDARY_LABELS.items()
                  if kind.startswith('Flap ')})
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
        if point.get('vertical_edge'):
            edge=point.get('edge_kind')
            label=edges.get(edge,label)
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


def prepare_exports(data, *, started_at=None):
    if 'plot_max_turn' not in data:enrich(data)


    cleaned={}
    def performance_only(value):
        if not isinstance(value,(dict,list)):
            if isinstance(value,(float,np.floating)) and not math.isfinite(value):
                raise ValueError('Nonfinite numerical export')
            return value
        identity=id(value)
        if identity in cleaned:return cleaned[identity]
        if isinstance(value,dict):
            result={k:performance_only(v) for k,v in value.items() if k not in ('sticks','trim') and not k.startswith('_')}
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
    data=performance_only(data)
    points={}


    chart={k:v for k,v in data.items() if k!='aircraft'};chart['aircraft']=[]
    for aircraft in data['aircraft']:
        item={k:v for k,v in aircraft.items() if k not in ('columns','surface','points','sustained','boundary_columns')}
        points[aircraft['id']]=[performance_only(p) for p in aircraft['points']+aircraft.get('sustained',[])]
        def brief(point,index):
            return dict({k:point[k] for k in ['speed_kmh','load_g','turn_dps','valid','ps_mps','reasons','flaps_percent','flaps_requested_percent']},detail_index=index)
        item['points']=[brief(p,i) for i,p in enumerate(aircraft['points'])]
        item['sustained']=[brief(p,len(aircraft['points'])+i) for i,p in enumerate(aircraft.get('sustained',[]))]
        chart['aircraft'].append(item)
    add_boundary_hover(chart,data);chart['boundary_hover_ready']=True
    if started_at is not None:
        chart['calculation_elapsed_s']=chart['elapsed_s']
        chart['elapsed_s']=time.monotonic()-started_at
    return dict(data=data,chart=chart,points=points)


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
