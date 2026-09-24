"""Prepared variable-wing geometry: native 1019e1670 / 101a27c50.

Interpolate AFTER each wing's property loader (not its raw JSON). In particular,
chord-scaled arms and polar aspect ratios are already prepared at each endpoint.
The left endpoint supplies the AoaShiftAdd, flap and Cm grids, as in the binary.
"""
from component_assembly import f32, add, sub, mul
from control_mixer import prepare_rows, curve
from polar_runtime import make_runtime, interpolate
from wing_model import selected_geometry


def prepare_plane(plane):
    g=selected_geometry({'Aerodynamics':{'WingPlane':plane}})
    g.update(sweep=f32(plane['SweptAngle']),taper=f32(plane['TaperRatio']),
             downwash_type=int(plane.get('DownwashType',1)),
             downwash_coefficient=f32(plane.get('DownwashCoeff',1.)))
    strength=plane.get('Strength',{})
    g['strength']=dict(force=list(map(f32,strength.get('CritOverload',[-2147440000.,2147440000.]))),
                       ias=mul(f32(strength.get('VNE',300.)),f32(1/3.6)),mach=f32(strength.get('MNE',.8)))
    family=[]
    blocks=[plane['FlapsPolar'+str(i)] for i in range(16) if 'FlapsPolar'+str(i) in plane]
    if not blocks and 'Polar' in plane:blocks=[plane['Polar']]
    for p in blocks:
        x=f32(p.get('Flaps',0.))
        if not family or sub(x,family[-1][0])>f32(4e-19):family.append((x,make_runtime(p,g['span'],g['area'])))
    if not family:raise ValueError('Wing has no prepared polar family')
    return dict(geometry=g,polars=family)


def linear_polar(family,x):
    """Ordinary prepared-family interpolation, without the flap sqrt mapping."""
    if len(family)==1 or x<=family[0][0]:return family[0][1]
    if x>=family[-1][0]:return family[-1][1]
    for (xa,a),(xb,b) in zip(family,family[1:]):
        if x<=xb:return interpolate(a,b,mul(sub(x,xa),f32(1/sub(xb,xa))))
    raise ValueError('Invalid flap family')


def scalar_table(points,x):
    if not points:return 0.
    if len(points)==1 or x<=points[0][0]:return points[0][1]
    if x>=points[-1][0]:return points[-1][1]
    for (xa,a),(xb,b) in zip(points,points[1:]):
        if x<=xb:return add(mul(sub(b,a),mul(sub(x,xa),f32(1/sub(xb,xa)))),a)
    raise ValueError('Invalid scalar table')


def blend(a,b,k):
    k=f32(k);ga,gb=a['geometry'],b['geometry'];near=ga if k<.5 else gb
    def lerp(x,y):return add(mul(sub(y,x),k),x)
    g={key:lerp(ga[key],gb[key]) for key in ['span','incidence','sweep','taper','dihedral',
          'sine_aos','v_focus','aoa_shift','downwash_coefficient']}
    for key in ['arm','spin_loss']:g[key]=[lerp(x,y) for x,y in zip(ga[key],gb[key])]
    g['areas']=[[lerp(x,y) for x,y in zip(sa,sb)] for sa,sb in zip(ga['areas'],gb['areas'])]
    left,right=g['areas'];g['area']=add(add(add(add(add(left[1],left[0]),left[2]),right[0]),right[1]),right[2])
    g['shifts']={key:[lerp(x,y) for x,y in zip(ga['shifts'][key],gb['shifts'][key])] for key in ga['shifts']}
    g['aoa_shift_add']=[[x,lerp(y,scalar_table(gb['aoa_shift_add'],x))] for x,y in ga['aoa_shift_add']]
    for key in ['use_spin_loss','downwash_type']:g[key]=near[key]
    g['strength']={key:lerp(ga['strength'][key],gb['strength'][key]) for key in ['ias','mach']}
    g['strength']['force']=[lerp(x,y) for x,y in zip(ga['strength']['force'],gb['strength']['force'])]
    family=[(x,interpolate(p,linear_polar(b['polars'],x),k)) for x,p in a['polars']] if b['polars'] else a['polars']
    return dict(geometry=g,polars=family)


def prepare(fm):
    ad=fm['Aerodynamics'];family=[]
    for i in range(16):
        key='WingPlaneSweep'+str(i)
        if key in ad:family.append((f32(ad[key].get('Sweep',0.)),prepare_plane(ad[key])))
    if not family:return [(0.,prepare_plane(ad['WingPlane']))]
    if any(a[0]>=b[0] for a,b in zip(family,family[1:])):raise ValueError('Wing sweep knots must increase')
    return family


def select(family,sweep):
    sweep=f32(sweep)
    if len(family)==1 or sweep<=family[0][0]:return family[0][1]
    if sweep>=family[-1][0]:return family[-1][1]
    for (xa,a),(xb,b) in zip(family,family[1:]):
        if sweep<=xb:return blend(a,b,mul(sub(sweep,xa),f32(1/sub(xb,xa))))
    raise ValueError('Invalid wing sweep input')


def schedule(fm):
    ad=fm['Aerodynamics'];rows=[]
    if 'SweepAxisByMachAuto' in ad:rows=[(0.,ad['SweepAxisByMachAuto'])]
    else:
        for i in range(8):
            if 'SweepAxisByMachAuto'+str(i) in ad:
                x,*v=ad['SweepAxisByMachAuto'+str(i)];rows.append((x,v))
    return prepare_rows(rows or [(0.,[0.,0.,1.])])


def available(fm,rows,mach,mechanism_bounds=(0.,1.)):
    """Steady airborne sweep range from 101a4c88a..cb84.

    SemiAuto maps pilot demand into the Mach-dependent [minimum, maximum].
    Auto uses the middle column. Manual, when enabled, uses mechanism bounds.
    Rate limits affect transitions, not settled operating points.
    """
    modes=fm.get('AvailableControls',{}).get('HasSweepControlMode',{})
    minimum,automatic,maximum=curve(rows,f32(mach),3)
    lo,hi=map(f32,mechanism_bounds)
    if modes.get('Manual',False):return lo,hi
    clamp=lambda x:min(hi,max(lo,x))
    if modes.get('SemiAuto',False):return tuple(sorted((clamp(minimum),clamp(maximum))))
    if modes.get('Auto',False):return (clamp(automatic),)*2
    return (clamp(0.),)*2
