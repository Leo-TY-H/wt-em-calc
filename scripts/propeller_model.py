"""Four-station intact blade force kernel 101a07790 for the selected fighters.

This is not the full propeller: the caller must supply induced flow, pitch and
shaft speed. Governor, azimuth integration and transmission remain separate.
"""
import math
from collections import OrderedDict
from functools import lru_cache
from component_assembly import f32, add, sub, mul
from piston_model import div
from polar_runtime import CONST_KEYS, evaluate
from polar_f32 import calc_cl, calc_cd
from mach_cubic import all_coefficients


# Loaded blade polars are immutable. Retain their owner alongside each cache
# so object IDs cannot be reused underneath an entry. Both levels are bounded.
_blade_polars=OrderedDict()


def blade_polar(runtime,mach):
    """Reuse Mach coefficients at exact native float32 inputs, without bins."""
    identity=id(runtime)
    if identity not in _blade_polars:
        if len(_blade_polars)>=32:_blade_polars.popitem(last=False)
        _blade_polars[identity]=(runtime,OrderedDict())
    _blade_polars.move_to_end(identity)
    memo=_blade_polars[identity][1]
    if mach not in memo:
        polar=evaluate(runtime,mach)
        if runtime['mode']==2:polar.update(legacy_stall_shape())
        if len(memo)>=1024:memo.popitem(last=False)
        memo[mach]=polar
    else:memo.move_to_end(mach)
    return memo[mach]


@lru_cache(maxsize=1)
def legacy_stall_shape():
    """101a08760 initializes the shared mode-2 blade stall shape.

    The blade kernel replaces four prepared polar fields with these defaults,
    even when its authored polar has different critical lift/angles.
    """
    base=list(map(f32,[55.5,.075,3.,.02222222089767456,36.,.01,-1.09,1.09,
                      .4,17.,-17.,1.4,-.9,.012,1.,1.]))
    polar=evaluate(dict(base=base,mode=0),0.)
    return {k:polar[k] for k in ['aoaLineH','aoaLineL','parabCyCoeffH','parabCyCoeffL']}


def prepare(fm):
    """Static geometry/polar adapter; native loader defaults are explicit."""
    modern = 'PropellerType0' in fm
    prop = fm['PropellerType0'] if modern else fm['Engine0']['Propellor']
    g = prop['Geometry'] if modern else prop
    radius = f32(g['Radius' if modern else 'AdvancedPropRadius'])
    blades = g['NumBlades']
    width = [f32(g[('BladeWidth' if modern else 'PropWidth')+str(i)]) for i in range(4)]
    twist = [mul(f32(g[('BladePitch' if modern else 'PropPhi')+str(i)]), f32(.01745329238474369)) for i in range(4)]
    if modern:
        area = mul(mul(add(add(width[3], width[1]), add(width[2], width[0])), f32(blades)), mul(f32(.2),mul(radius,radius)))
        aspect = div(mul(mul(radius, radius), f32(blades)), area)
    else:
        area = mul(mul(mul(mul(add(add(add(width[3], width[2]), width[1]), width[0]), radius), radius), f32(.2)), f32(blades))
        polar_area = mul(mul(add(width[3],add(width[2],add(width[1],width[0]))),mul(f32(.2),mul(radius,radius))),f32(blades))
        aspect = div(mul(mul(radius,radius),f32(blades)),polar_area)
        # The legacy prop's missing Oswald defaults to 55.5/aspect, retaining
        # effective lambda approximately 55.5. It is not a wing-polar default.
    props = dict(lineClCoeff=.075, Cl0=.4, alphaCritHigh=17., alphaCritLow=-17.,
                 ClCritHigh=1.4, ClCritLow=-.9, CdMin=.012, AfterCritParabAngle=3.,
                 AfterCritDeclineCoeff=.02222222089767456, AfterCritMaxDistanceAngle=36.,
                 CxAfterCoeff=.01, ClAfterCritHigh=1.09, ClAfterCritLow=-1.09)
    props.update(prop['Polar'])
    if 'ClAfterCritLow' not in prop['Polar']:
        props['ClAfterCritLow'] = -props['ClAfterCritHigh']
    lam = mul(aspect, f32(props.get('OswaldsEfficiencyNumber', div(55.5, aspect))))
    defaults = [[.6,1,7,-5.2,1], [.65,.97,6.7,-3.7,1], [.3,1,.32,-.44,.25],
                [.3,1,.4,-.2,.25], [.6,1.5,2,1.1,5], [0,0,0,0,0], [0,1,1,0,1]]
    rows = [[f32(props.get(k+str(i+1), d)) for k,d in zip(
        ['MachCrit','MachMax','MultMachMax','MultLineCoeff','MultLimit'], row)]+[0.]*4
        for i,row in enumerate(defaults)]
    runtime = dict(base=[lam]+[f32(props[k]) for k in CONST_KEYS]+[1.,area],
                   mode=props.get('MachFactor',2), combined=props.get('CombinedCl',True),
                   mach=rows, cm=[(0.,0.,[0.,0.])])
    if runtime['mode'] != 3:
        raise ValueError('This adapter is restricted to the selected mode-3 props')
    for row,c in zip(rows,all_coefficients(runtime)): row[5:] = c
    return dict(radius=radius, blades=blades, width=width, twist=twist, polar=runtime)


def blade_forces(p, omega, pitch, axial, tangential, crossflow, axial_gradient,
                 density, sound_speed, angular_flow=0.):
    """Complete baseline 101a07790 return: thrust N, resisting shaft torque Nm.

    Speed/pitch/flow are delivered states, not engine settings. Even aircraft
    speed zero can yield nonzero prop inflow supplied by the parent solver.
    """
    omega,pitch,axial,tangential,crossflow,axial_gradient,density,sound_speed,angular_flow = map(
        f32,(omega,pitch,axial,tangential,crossflow,axial_gradient,density,sound_speed,angular_flow))
    if sound_speed <= 0.: raise ValueError('Positive sound speed required')
    thrust = torque = 0.
    area_unit = mul(mul(p['radius'], p['radius']), f32(.2))
    half_density = mul(density,.5)
    cross_sq = mul(mul(crossflow,crossflow),.5)
    for station,twist,width in zip([f32(.35), f32(.55), .75, f32(.9500000476837158)], p['twist'],p['width']):
        r = mul(station,p['radius'])
        u = add(mul(r,axial_gradient),axial)
        v = mul(r,sub(omega,angular_flow))
        phi = f32(math.atan2(u,v)) if abs(omega)>=f32(1e-5) else f32(math.pi/2)
        alpha = mul(add(sub(pitch,phi),twist), f32(57.2957763671875))
        vt = add(v,tangential)
        speed_sq = add(add(mul(vt,vt),cross_sq),mul(u,u))
        speed = f32(math.sqrt(speed_sq))
        polar = blade_polar(p['polar'],div(speed,sound_speed))
        force_unit = mul(mul(speed_sq,half_density),mul(width,area_unit))
        lift = mul(mul(calc_cl(polar,alpha),force_unit),polar['clKq'])
        drag = mul(mul(calc_cd(polar,alpha),force_unit),polar['kq'])
        inv_speed = div(1.,speed)
        un,vn = mul(u,inv_speed),mul(v,inv_speed)
        thrust = sub(add(mul(lift,vn),thrust),mul(un,drag))
        torque = add(mul(add(mul(lift,un),mul(vn,drag)),r),torque)
    return dict(thrust=mul(thrust,f32(p['blades'])), torque=mul(torque,f32(p['blades'])))


def properties(fm):
    p=prepare(fm)
    modern='PropellerType0' in fm
    raw=fm['PropellerType0'] if modern else fm['Engine0']['Propellor']
    g=raw['Geometry'] if modern else raw
    gov=raw['Governor'] if modern else raw
    mass=raw['Mass'] if modern else raw
    inertia_base=mul(mul(mul(f32(mass['Diameter']),f32(mass['Diameter'])),f32(mass['Mass'])),f32(mass['InertiaMomentCoeff']))
    weights=[mul(w,s) for w,s in zip(p['width'],map(f32,[.122499995,.3025,.5625,.9025001]))]
    weighted=add(add(weights[3],weights[1]),add(weights[2],weights[0]))
    mean=div(add(add(mul(weights[3],p['twist'][3]),mul(weights[2],p['twist'][2])),add(mul(weights[1],p['twist'][1]),mul(weights[0],p['twist'][0]))),weighted)
    dr=mul(p['radius'],.2)
    cw=[f32(math.cos(t)) for t in p['twist']]
    width_proj=mul(add(add(mul(mul(p['width'][3],cw[3]),dr),add(mul(mul(p['width'][1],cw[1]),dr),mul(mul(cw[0],p['width'][0]),dr))),mul(mul(p['width'][2],dr),cw[2])),1.25)
    if not modern:
        terms=[mul(mul(cw[3],dr),p['width'][3]),mul(mul(cw[2],p['width'][2]),dr),mul(mul(cw[1],p['width'][1]),dr),mul(mul(cw[0],p['width'][0]),dr)]
        width_proj=mul(add(terms[0],add(terms[1],add(terms[2],terms[3]))),1.25)
    p.update(mean_twist=mean,projected_width=width_proj,
             feather_pitch=add(mul(add(add(p['twist'][3],p['twist'][2]),add(p['twist'][1],p['twist'][0])),-.25),f32(1.483529806137085)),
             critical_ias=mul(f32(3600.),f32(.2777778)),
             inertia=mul(inertia_base,f32(1/12)),inverse_inertia=div(12.,inertia_base),
             inertia_coefficient=f32(mass['InertiaMomentCoeff']),mass=f32(mass['Mass']),diameter=f32(mass['Diameter']),
             pitch_min=mul(gov['PitchMin' if modern else 'PhiMin'],f32(math.pi/180)),
             pitch_max=mul(gov['PitchMax' if modern else 'PhiMax'],f32(math.pi/180)),
             aoa0=mul(gov['Aoa0' if modern else 'PhiAlpha0'],f32(math.pi/180)),
             governor=gov['GovernorType'],governor_speed=mul(gov['GovernorSpeed'],10.),
             governor_fast=gov.get('GovernorFast',False),
             min_omega=mul(gov['GovernorMinParam'],f32(.104719758)),
             max_omega=mul(gov['GovernorMaxParam'],f32(.104719758)),
             inv_max_omega=div(f32(9.549296379),gov['GovernorMaxParam']),
             boost_omega=mul(gov['GovernorAfterburnerParam'],f32(.104719758)),
             reduction=f32(fm['Transmission0']['PropellerReductor0'] if modern else raw['Reductor']),
             position=list(map(f32,fm['Propeller0']['Pos'] if modern else fm['Engine0']['PropPos'])),
             direction=g['RotationDirection'] if 'RotationDirection' in g else (fm['Propeller0']['Geometry']['RotationDirection'] if modern else g['Direction']),
             auto_allowed=raw['Controls']['HasAutoPitchControl'] if modern else raw['AllowAutoProp'])
    p['pitch_command_report'] = p['auto_allowed'] if p['governor'] in [1,2] else p['governor'] not in [0,3,4,6,7,9,10]
    return p
