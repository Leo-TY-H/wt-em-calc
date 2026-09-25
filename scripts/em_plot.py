"""Plot preparation and standalone scientific exports for reconstructed EM data."""
import csv
import io
import json
import orjson
import math
import time
import textwrap
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np
import contourpy
from scipy.interpolate import PchipInterpolator

LEVELS=[-300.,-200.,-100.,-50.,0.,50.,100.,200.,300.]
PROP_LEVELS=[-300.,-200.,-100.,-75.,*map(float,range(-50,51,5)),75.,100.,200.,300.]
SYMBOL_FONT=Path(__file__).resolve().parents[1]/'app/fonts/wt-symbols.ttf'
font_manager.fontManager.addfont(SYMBOL_FONT)
SYMBOL_FAMILY=font_manager.FontProperties(fname=SYMBOL_FONT).get_name()


def nullable_grid(values):
    """Convert masked display grids in bulk, preserving every finite float."""
    array=np.asarray(values);out=array.astype(object)
    out[~np.isfinite(array)]=None
    return out.tolist()


def propeller_plot(aircraft):
    return aircraft.get('propulsion') in ('piston','turboprop','mixed')


def contour_levels(aircraft):
    return PROP_LEVELS if propeller_plot(aircraft) else LEVELS


def heatmap_column(y,z,valid,grid):
    """Original linear cell interpolation, batched over contiguous valid runs.

    A missing cell still separates runs; isolated samples do not paint a band.
    Keep the old per-cell behavior for non-increasing diagnostic coordinates.
    """
    out=np.full(len(grid),np.nan)
    for run in np.split(valid,np.where(np.diff(valid)>1)[0]+1):
        if len(run)<2:continue
        if np.all(np.diff(y[run])>0):
            select=(grid>=y[run[0]])&(grid<=y[run[-1]])
            out[select]=np.interp(grid[select],y[run],z[run])
        else:
            for a,b in zip(run,run[1:]):
                select=(grid>=y[a])&(grid<=y[b])
                out[select]=np.interp(grid[select],y[a:b+1],z[a:b+1])
    return out


def matrices(data, aircraft):
    from em_continuous_boundary import mask_surface
    if 'surface' in aircraft:
        s=aircraft['surface'];x,y=np.meshgrid(s['x'],s['fraction'])
        y=np.array(s['turn_dps']);z=np.ma.masked_invalid(np.array(s['z'],dtype=float))
        return x,y,mask_surface(aircraft,x,y,z)
    nx=len(data['speeds_kmh']); ny=len(data['loads_g'])
    points=aircraft['points']
    x=np.array([p['speed_kmh'] for p in points]).reshape(ny,nx)
    y=np.array([p['turn_dps'] for p in points]).reshape(ny,nx)
    z=np.ma.array(np.array([p['ps_mps'] for p in points]).reshape(ny,nx),
                  mask=np.array([not p['valid'] for p in points]).reshape(ny,nx))
    return x,y,mask_surface(aircraft,x,y,z)


def smooth_surface(data,aircraft):
    """Shape-preserving surface inside each connected, sampled trim envelope.

    Speed-midpoint and load-midpoint errors are checked during sampling. The
    display grid has independent resolution; it is not advertised as solved
    points. Empty columns break the interpolant and all plotted curves.
    """
    from em_sampling import speed_interpolate
    # The contour engine interpolates linearly inside each display cell. A
    # coarse speed raster can exceed the SEP tolerance at a sharp but continuous
    # control/polar transition even when the checked cubic is accurate.
    columns=aircraft['columns'];nx=2*data['settings']['surface_resolution']-1;nf=1201
    outline=aircraft.get('boundary_columns',columns)
    if data.get('preview'):nx=min(nx,241);nf=121
    xs=np.linspace(data['settings']['speed_min_kmh'],data['settings']['speed_max_kmh'],nx)
    # Exact solved speed knots include the low-speed endpoint. A uniform
    # display raster alone can omit that endpoint and visibly detach the edge.
    xs=np.unique(np.concatenate([xs,[c['speed_kmh'] for c in columns],[c['speed_kmh'] for c in outline]]));nx=len(xs)
    # A subpixel unresolved interval may contain no regular mesh vertex.
    # Without an explicit masked vertex, contourpy connects its two solved
    # endpoints and paints the very interpolant that failed verification.
    uncertain=aircraft.get('interpolation',{}).get('unresolved_speed_intervals',[])
    separators=[]
    for lo,hi in uncertain:
        cuts=[lo,*sorted(c['speed_kmh'] for c in columns if lo<c['speed_kmh']<hi),hi]
        separators.extend(a+(b-a)*.5 for a,b in zip(cuts,cuts[1:]) if a<b)
    if separators:xs=np.unique(np.concatenate([xs,separators]));nx=len(xs)
    # Retain the solved brackets around fixed-sweep speed restrictions in
    # display/export coordinates too, rather than truncating at a raster cell.
    edges=[edge+delta for span in aircraft.get('sweep_excluded_speeds_kmh',[]) for edge in span for delta in [-.01,.01]
           if data['settings']['speed_min_kmh']<edge+delta<data['settings']['speed_max_kmh']]
    if edges:xs=np.unique(np.concatenate([xs,edges]));nx=len(xs)
    f=1.-(1.-np.linspace(0.,1.,nf))**2;z=np.full((nf,nx),np.nan);turn=np.zeros((nf,nx));caps=np.full(nx,np.nan)
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
    # Fit the mesh to both verified edges. Uniform physical-load rows stop
    # below a sloping cap and leave contours detached from the separately
    # drawn boundary; every column now has an actual edge vertex. The speed
    # interpolant and outline use exactly the same verified support.
    from em_surface import envelope_limits,mesh_limits
    limits=envelope_limits(columns,xs)
    fitted=np.isfinite(limits).all(axis=1)
    caps[fitted]=limits[fitted,1]
    # Mesh coordinates are not envelope certificates. Using the chart-wide
    # load ceiling at an unresolved column stretched cells abruptly and made
    # contourpy invent SEP needles between correct physical samples.
    sampled_limits=mesh_limits(columns,xs)
    sampled=np.isfinite(sampled_limits).all(axis=1)
    lower=np.where(fitted,limits[:,0],np.where(sampled,sampled_limits[:,0],1.))
    upper=np.where(np.isfinite(caps),caps,np.where(sampled,sampled_limits[:,1],lower))
    fraction=np.linspace(0.,1.,nf)
    lower_turn=np.sqrt(np.maximum(0.,lower*lower-1.))
    upper_turn=np.sqrt(np.maximum(0.,upper*upper-1.))
    loads=np.sqrt(1.+(lower_turn[None,:]+fraction[:,None]*(upper_turn-lower_turn)[None,:])**2)
    loads[0]=lower;loads[-1]=upper
    z=speed_interpolate(columns,xs,loads)
    # A speed cell that exhausted adaptive checks has no certified interior
    # interpolant. Retain its solved endpoint columns and mask only the
    # unverified open interval between them.
    solved_speeds=np.isin(xs,[c['speed_kmh'] for c in columns])
    for lo,hi in aircraft.get('interpolation',{}).get('unresolved_speed_intervals',[]):
        # An uncertain upper limit does not invalidate independently solved
        # interior samples at this exact speed. Keep their load curve; its
        # rejected loads remain masked by speed_interpolate itself.
        z[:,(xs>lo)&(xs<hi)&~solved_speeds]=np.nan
    # A boundary transition can leave the upper band uncertain while the
    # fixed-load interior passes independent trims at two interior speeds.
    # Restore exactly that checked band; failed or unchecked loads stay masked.
    from em_speed_seam import plot_values
    for certificate in aircraft.get('interpolation',{}).get('certified_speed_interiors',[]):
        lo,hi=certificate['speed_interval_kmh'];mask=(xs>lo)&(xs<hi)&~solved_speeds
        z[:,mask]=plot_values(columns,certificate,xs[mask],loads[:,mask])
    turn=np.degrees(9.8100004196167*np.sqrt(loads**2-1.)/(xs[None,:]/3.6))
    for i,x in enumerate(xs):
        cap=caps[i]
        index=max(0,min(len(outline)-2,int(np.searchsorted([c['speed_kmh'] for c in outline],x))-1))
        verified=all(c['boundary_status'] in ('verified limit','plot ceiling') for c in outline[index:index+2])
        if not solved_speeds[i] and any(lo<x<hi for lo,hi in uncertain):verified=False
        rate=math.degrees(9.8100004196167*math.sqrt(max(0.,cap*cap-1.))/(x/3.6)) if np.isfinite(cap) else None
        boundary.append(dict(speed_kmh=float(x),turn_dps=rate if verified else None,
                             at_plot_ceiling=bool(data['settings']['max_load_g'] is not None and np.isfinite(cap) and abs(cap-data['settings']['max_load_g'])<1e-5)))
    # Every independently verified boundary knot survives, including a lone
    # endpoint next to an empty column or an interior interpolation gap.
    exact={c['speed_kmh']:c for c in outline}
    for point in boundary:
        column=exact.get(point['speed_kmh'])
        if column and column['boundary_status'] in ('verified limit','plot ceiling'):
            point['turn_dps']=column['boundary']['turn_dps']
    edge=aircraft.get('low_speed_edge')
    if edge:
        at=next((i for i,p in enumerate(boundary) if p['speed_kmh']==edge['speed_kmh'] and p['turn_dps'] is not None),None)
        if at is not None:
            # This explicit solid vertical outline does not fill unverified
            # performance below a disconnected lower equilibrium branch.
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
    aircraft['boundary']=boundary
    aircraft['numerical_boundaries']=[dict(speed_kmh=c['speed_kmh'],turn_dps=c['boundary']['turn_dps'],load_g=c['boundary']['load_g'])
        for c in outline if c['boundary'] and c['boundary_status']=='unresolved numerical boundary']
    aircraft['numerical_gaps']=[dict(speed_kmh=c['speed_kmh'],**gap,
        turn_dps=math.degrees(9.8100004196167*math.sqrt(gap['load_g']**2-1.)/(c['speed_kmh']/3.6)))
        for c in columns for gap in c.get('numerical_gap_brackets',[])]
    aircraft['surface']=dict(x=xs.tolist(),fraction=fraction.tolist(),load_g=loads.tolist(),mesh='verified boundary and uniform turn fraction',turn_dps=turn.tolist(),
                             z=nullable_grid(z))


def enrich(data):
    """Contour only adjacent valid cells; never bridge a rejected trim point."""
    max_turn=max((p['turn_dps'] for a in data['aircraft'] for p in a['points'] if p['valid']),default=20.)
    max_turn=max(max_turn,max((p['turn_dps'] for a in data['aircraft']
                              for p in a.get('continuous_pull_boundary',[]) if p['turn_dps'] is not None),default=0.))
    data['plot_max_load_g']=max(1.1,max((p['load_g'] for a in data['aircraft'] for p in a['points'] if p['valid']),default=1.1))
    data['plot_max_turn']=max(10.,math.ceil((max_turn+1)/5)*5.)
    regular_y=np.linspace(0.,data['plot_max_turn'],161 if data.get('preview') else 401)
    for aircraft in data['aircraft']:
        if 'columns' in aircraft:smooth_surface(data,aircraft)
        x,y,z=matrices(data,aircraft); paths=[]
        if z.count()>3 and np.ma.max(z)>np.ma.min(z):
            # Matplotlib uses this same contour engine. Interactive results
            # need its paths, not a temporary figure, axes or text layout.
            contour=contourpy.contour_generator(x=x,y=y,z=z,name='mpl2014',corner_mask=False)
            for level in contour_levels(aircraft):
                for segment in contour.lines(level)[0]:
                    if len(segment)>1:paths.append(dict(level=float(level),x=segment[:,0].tolist(),y=segment[:,1].tolist()))
        heat=np.full((len(regular_y),x.shape[1]),np.nan)
        boundary=[];mask=np.ma.getmaskarray(z)
        for i in range(x.shape[1]):
            valid=np.where(~mask[:,i])[0]
            if len(valid):
                j=valid[-1]
                root_turn=max((p['turn_dps'] for p in aircraft.get('sustained',[]) if p['speed_kmh']==x[j,i]),default=0.)
                boundary.append(dict(speed_kmh=float(x[j,i]),turn_dps=float(y[j,i]),
                                     at_plot_ceiling=bool(j==x.shape[0]-1)))
                boundary[-1]['turn_dps']=max(boundary[-1]['turn_dps'],root_turn)
            else:boundary.append(dict(speed_kmh=float(x[0,i]),turn_dps=None,at_plot_ceiling=False))
            heat[:,i]=heatmap_column(y[:,i],z.data[:,i],valid,regular_y)
        aircraft['contours']=paths
        if 'surface' in aircraft:
            # Every displayed isoline uses the same checked surface and edge
            # vertices. Interpolating only the discrete solved Ps=0 roots cut
            # that line off at its last speed column, before the physical edge.
            zeros=[p for p in paths if p['level']==0.]
            aircraft['sustained_curve']=dict(
                x=[x for p in zeros for x in [*p['x'],None]],
                y=[y for p in zeros for y in [*p['y'],None]])
        aircraft['ps_color_limit_mps']=50. if propeller_plot(aircraft) else 300.
        if aircraft.get('continuous_pull_boundary'):
            aircraft['boundary']=aircraft['continuous_pull_boundary']
        elif 'surface' not in aircraft:aircraft['boundary']=boundary
        aircraft['heatmap']=dict(x=x[0,:].tolist(),y=regular_y.tolist(),z=nullable_grid(heat))
    return data


def preview_payload(data):
    """Transient display data; never writes completed-result/export markers."""
    enrich(data)
    chart={k:v for k,v in data.items() if k!='aircraft'}
    chart['aircraft']=[{k:v for k,v in a.items() if k not in ('columns','surface','boundary_columns')}
                       for a in data['aircraft']]
    return chart


def export_csv(data):
    condition_fields=['torque_gyro','engine_control_mode','altitude_m','fuel_percent','throttle','afterburner','extra_mass_kg','structural_limits','timestep_hz']
    out=io.StringIO();fields=['aircraft','aircraft_id','aircraft_name','kind']+condition_fields+['speed_kmh','load_g','turn_dps','ps_mps','ps_continuous_mps',
          'valid','converged','reasons','alpha_deg','bank_deg','sideslip_deg','sideslip_attitude_deg','ias_kmh','mach','force_error_g',
          'angular_error_rad_s2','history_error','stall_margin_deg','vertical_step_velocity_mps',
          'commands','authority_margin','force_n','moment_nm','engine_force_n','sweep_percent','sweep_available_percent',
          'flaps_requested_percent','flaps_percent','gear_percent','instructor_enabled','instructor','prolonged_pull','propulsion']
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
                row.update({k:cfg.get(k,'optimized') if k=='engine_control_mode' else cfg.get(k,True) if k=='torque_gyro' else cfg[k] for k in condition_fields})
                for key,value in row.items():
                    if isinstance(value,(list,dict)):row[key]=json.dumps(value,separators=(',',':'))
                writer.writerow(row)
    return out.getvalue()


def export_figure(data, path, selected=None):
    """Curvilinear native samples; SVG, PNG and PDF use the same scientific plot."""
    aircraft=[a for a in data['aircraft'] if selected is None or a['id']==selected]
    # Symbols must precede DejaVu: many are ordinary Unicode block characters
    # that the game's font deliberately draws as national insignia. SVG paths
    # keep those shapes portable without requiring the font on the viewer's PC.
    with plt.rc_context({'font.family':[SYMBOL_FAMILY,'DejaVu Sans'],
                         'font.size':10,'svg.fonttype':'path'}):
        height=7+max(0,len(aircraft)-2)*.35
        fig,ax=plt.subplots(figsize=(11,height))
        fig.subplots_adjust(left=.09,right=.97,bottom=1.05/height,top=1-(.98+max(0,len(aircraft)-2)*.35)/height)
        for a in aircraft:
            color={'f_16a_block_15_adf':'#007f92','f_16xl':'#007f92','j6k1':'#bd581c','saab_jas39c':'#bd581c'}.get(a['id'],a['color'])
            x,y,z=matrices(data,a)
            if z.count()>3 and np.ma.max(z)>np.ma.min(z):
                if len(aircraft)==1:
                    fill_levels=np.arange(-50,51,5) if propeller_plot(a) else np.arange(-400,401,25)
                    filled=ax.contourf(x,y,z,levels=fill_levels,cmap='RdYlBu',extend='both',alpha=.78,corner_mask=False)
                    fig.colorbar(filled,ax=ax,label='Specific excess power, native step (m/s)')
                contour=ax.contour(x,y,z,levels=[n for n in contour_levels(a) if n!=0],colors=color if len(aircraft)>1 else '#526174',linewidths=.7,linestyles='dashed',corner_mask=False)
                ax.clabel(contour,inline=True,fontsize=7,fmt=lambda v:f'SEP {v:g} m/s')
            boundary=a['boundary']
            prefix='' if len(aircraft)==1 and a.get('continuous_pull_boundary') else a['name']+' '
            ax.plot([p['speed_kmh'] for p in boundary],[np.nan if p['turn_dps'] is None else p['turn_dps'] for p in boundary],
                    '-',color=color,lw=1.5,zorder=3.5,clip_on=False,label=prefix+('Full-pitch Instructor pull (experimental)' if a.get('continuous_pull_boundary') else 'experimental Instructor boundary' if a.get('settings',data['settings']).get('instructor') else 'verified feasible boundary'))
            if data.get('show_numerical_diagnostics') and a.get('numerical_boundaries'):
                ax.scatter([p['speed_kmh'] for p in a['numerical_boundaries']],[p['turn_dps'] for p in a['numerical_boundaries']],
                           marker='x',color='#b45a12',s=22,label=a['name']+' unresolved numerical boundary')
            if data.get('show_numerical_diagnostics') and a.get('numerical_gaps'):
                ax.scatter([p['speed_kmh'] for p in a['numerical_gaps']],[p['turn_dps'] for p in a['numerical_gaps']],
                           marker='o',facecolors='none',edgecolors='#b45a12',s=16,label=a['name']+' local equilibrium gaps')
            # None at missing speeds prevents an apparent sustained segment
            # through unvalidated/invalid grid columns.
            roots={p['speed_kmh']:p for p in a.get('sustained',[])}
            root_curve=a.get('sustained_curve',dict(x=data['speeds_kmh'],y=[roots[v]['turn_dps'] if v in roots else np.nan for v in data['speeds_kmh']]))
            ax.plot(root_curve['x'],root_curve['y'],
                    color=color,lw=2.8,linestyle='--',label=prefix+'Ps = 0')
        for n in [2,4,6,9,12,16]:
            if n>data['plot_max_load_g']:continue
            v=np.array(data['speeds_kmh']); rate=np.degrees(9.8100004196167*np.sqrt(n*n-1)/(v/3.6))
            ax.plot(v,rate,color='#999999',alpha=.2,lw=.6)
        cfg=data['settings']
        conditions=[]
        for a in aircraft:
            c=a.get('settings',cfg)
            sweep=f" · sweep {c.get('sweep_percent',0):g}%" if a.get('has_sweep') else ''
            mode=('Instructor steady AoA (approximation)' if a.get('instructor_approximation',{}).get('kind')=='steady AoA schedule'
                  else 'Instructor experimental') if c.get('instructor') else 'Instructor off'
            if propeller_plot(a):mode+=' · experimental propeller · '+('automatic engines' if c.get('engine_control_mode')=='automatic' else 'idealized manual engines')+' · radiators closed · torque/gyro '+('on' if c.get('torque_gyro',True) else 'off (RB)')
            conditions.append(textwrap.fill(f"{a['name']}: {c['altitude_m']:g} m · {c['fuel_percent']:g}% fuel · throttle {c['throttle']*100:g}% · requested flaps {c.get('flaps_percent',0):g}%{sweep} · {mode}",width=145,break_long_words=False))
        ax.set_aspect('auto' if any(a.get('continuous_pull_boundary') for a in aircraft) else 20.,adjustable='box')
        ax.set(xlim=(cfg['speed_min_kmh'],cfg['speed_max_kmh']),ylim=(0,data['plot_max_turn']),
               xlabel='True airspeed (km/h)',ylabel='Turn rate (°/s)')
        # The fixed engineering aspect can make the axes narrow and move
        # their center next to the colorbar. Center long configuration labels
        # on the whole figure so the approximation label is never clipped.
        fig.suptitle('War Thunder · reconstructed EM diagram · Ps (m/s)\n'+'\n'.join(conditions),
                     x=.5,y=.98,fontsize=9)
        ax.grid(alpha=.15);ax.legend(loc='upper left' if any(a.get('continuous_pull_boundary') for a in aircraft) else 'upper right',fontsize=8)
        fig.text(.5,.05,'Ps: native-step energy rate · per-aircraft conditions above · fixed fuel / intact aircraft · positive-AoA stall limit',ha='center',fontsize=7,color='#596273')
        fig.text(.5,.025,'Numerical reconstruction; selected original-code checks pass; no live-flight validation.',ha='center',fontsize=7,color='#596273')
        fig.savefig(path,dpi=180);plt.close(fig)


def add_boundary_hover(chart, data):
    """Presentation-only Ps along the existing refined boundary coordinates.

    Use the saved boundary equilibria and the same contiguous column groups as
    smooth_surface. No equilibrium is solved, no boundary coordinate changes,
    and missing plotted segments stay missing. Off-knot Ps is explicitly marked
    as interpolation; it is not taken from a nearby interior heatmap cell.
    """
    sources={a['id']:a for a in data['aircraft']}
    for aircraft in chart['aircraft']:
        source=sources[aircraft['id']]
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
            point['ps_mps']=None;point['ps_interpolated']=False
            if point.get('vertical_edge') and point['turn_dps']==0.:
                column=next((c for c in outline if c['speed_kmh']==point.get('sample_speed_kmh',point['speed_kmh'])),None)
                level=next((p for p in column['points'] if p['valid'] and p['load_g']==1.),None) if column else None
                if level:point['ps_mps']=level['ps_mps']
        for group in groups:
            speeds=[c['speed_kmh'] for c in group]
            powers=[c['boundary']['ps_mps'] for c in group]
            exact=dict(zip(speeds,powers))
            curve=PchipInterpolator(speeds,powers,extrapolate=False) if len(group)>1 else None
            for point in aircraft['boundary']:
                speed=point.get('sample_speed_kmh',point['speed_kmh'])
                if point.get('vertical_edge') and point['turn_dps']==0.:continue
                if point['turn_dps'] is None or not speeds[0]<=speed<=speeds[-1]:continue
                value=exact.get(speed)
                if value is None and curve is not None:value=float(curve(speed))
                if value is not None and np.isfinite(value):
                    point['ps_mps']=value;point['ps_interpolated']=speed not in exact
                    near=min(group,key=lambda c:abs(c['speed_kmh']-speed))
                    if near.get('boundary_reason')=='trim fold' and not point.get('vertical_edge'):
                        point['edge_kind']='Trim limit · maximum load on the connected equilibrium branch'
    return chart


def write_exports(data, directory, *, figures=True, started_at=None):
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
    ready=directory/'ready.json';ready.unlink(missing_ok=True)
    if 'plot_max_turn' not in data:enrich(data)
    # Allocation remains available inside the solver/native validator, but is
    # not a performance output. Omit it from every nested exported point.
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
        elif isinstance(value[0],(int,float,np.number)):
            try:
                total=sum(value)
                if not math.isfinite(total) and any(not math.isfinite(v) for v in value):
                    raise ValueError('Nonfinite numerical export')
                result=value
            except TypeError:
                # Mixed lists still need nested private/control fields removed.
                result=[performance_only(v) for v in value]
        else:result=[performance_only(v) for v in value]
        cleaned[identity]=result
        return result
    (directory/'data.json').write_bytes(orjson.dumps(performance_only(data),option=orjson.OPT_APPEND_NEWLINE|orjson.OPT_SERIALIZE_NUMPY))
    # Keep pan/zoom and initial page loading independent of the size of the
    # numerical audit. Rich point records are fetched only when inspected.
    chart={k:v for k,v in data.items() if k!='aircraft'};chart['aircraft']=[]
    for aircraft in data['aircraft']:
        item={k:v for k,v in aircraft.items() if k not in ('columns','surface','points','sustained','boundary_columns')}
        offsets=[];all_points=aircraft['points']+aircraft.get('sustained',[])
        with (directory/(aircraft['id']+'-points.jsonl')).open('wb') as out:
            for point in all_points:
                offsets.append(out.tell());out.write(orjson.dumps(performance_only(point),option=orjson.OPT_APPEND_NEWLINE|orjson.OPT_SERIALIZE_NUMPY))
        (directory/(aircraft['id']+'-offsets.json')).write_text(json.dumps(offsets,separators=(',',':')),encoding='utf-8')
        def brief(point,index):
            return dict({k:point[k] for k in ['speed_kmh','load_g','turn_dps','valid','ps_mps','reasons','flaps_percent','flaps_requested_percent']},detail_index=index)
        item['points']=[brief(p,i) for i,p in enumerate(aircraft['points'])]
        item['sustained']=[brief(p,len(aircraft['points'])+i) for i,p in enumerate(aircraft.get('sustained',[]))]
        chart['aircraft'].append(item)
    add_boundary_hover(chart,data);chart['boundary_hover_ready']=True
    (directory/'chart.json').write_bytes(orjson.dumps(chart,option=orjson.OPT_APPEND_NEWLINE|orjson.OPT_SERIALIZE_NUMPY))
    (directory/'samples.csv').write_text(export_csv(data),encoding='utf-8')
    if figures:
        for extension in ['svg','png','pdf']:export_figure(data,directory/f'diagram.{extension}')
    marker=directory/'ready.json.tmp'
    timing=dict(interactive=True,figures=figures)
    if started_at is not None:timing['total_elapsed_s']=time.monotonic()-started_at
    marker.write_text(json.dumps(timing)+'\n',encoding='utf-8');marker.replace(ready)
