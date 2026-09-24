"""Float32 port of 0x1019e60c0 and its table helpers.

Inputs are prepared runtime curves, not raw BLK: speed knots are m/s and
the outer 2D axis is density. Does not reconstruct the input controller.
"""
from component_assembly import f32, add, sub, mul


def curve(rows, x, width):
    """Rows (knot, inverse_next_interval, values); clamp outside the knots."""
    if not rows:
        return [0.0] * width
    if len(rows) == 1 or x <= rows[0][0]:
        return list(rows[0][2])
    if x >= rows[-1][0]:
        return list(rows[-1][2])
    for lo, hi in zip(rows, rows[1:]):
        if x <= hi[0]:
            t = mul(sub(x, lo[0]), lo[1])
            return [add(mul(sub(b, a), t), a) for a, b in zip(lo[2], hi[2])]
    raise AssertionError('Unsorted or nonfinite curve input')


def surface_curve(rows, density, speed):
    if not rows:
        return [0.0] * 3
    if len(rows) == 1 or density <= rows[0][0]:
        return curve(rows[0][2], speed, 3)
    if density >= rows[-1][0]:
        return curve(rows[-1][2], speed, 3)
    for lo, hi in zip(rows, rows[1:]):
        if density <= hi[0]:
            t = mul(sub(density, lo[0]), lo[1])
            a, b = curve(lo[2], speed, 3), curve(hi[2], speed, 3)
            return [add(mul(sub(v, u), t), u) for u, v in zip(a, b)]
    raise AssertionError('Unsorted or nonfinite surface input')


def axis_limits(props, inverted, density, speed):
    result = []
    for i, inv in enumerate(inverted):
        rows = props['tables_2d'][i]
        # An incomplete 2D family falls back to the entire 1D family.
        a, center, b = (surface_curve(rows, density, speed)
                        if rows and all(r[2] for r in rows)
                        else curve(props['tables_1d'][i], speed, 3))
        positive, negative = props['angles'][i]
        if inv:
            result.append([mul(b, negative),
                           mul(center, negative if center >= 0 else positive),
                           mul(a, -positive)])
        else:
            result.append([mul(a, positive),
                           mul(center, positive if center >= 0 else negative),
                           mul(b, -negative)])
    return result


def mix(props, commands, inverted, aoa_bias, density, speed, mach, arcade=False):
    """Return deflection, AoA shift, CL add, CD add, wing-AoA contribution.

    No deflection clamp is present in this kernel. Inputs are float32 values;
    inversion flags must be canonical booleans as produced by the caller.
    """
    limits = axis_limits(props, inverted, density, speed)
    contributions = []
    for command, inv, (positive, center, negative) in zip(commands, inverted, limits):
        slope = sub(center, positive) if command >= 0 else sub(negative, center)
        value = sub(center, mul(slope, command))
        contributions.append(-value if inv else value)
    deflection = add(add(contributions[1], contributions[0]), contributions[2])
    sensitivity = curve(props['sensitivity_curve'], mach, 1)[0]
    if arcade:
        sensitivity = mul(sensitivity, curve(props['arcade_curve'], mach, 1)[0])
    scaled = mul(sensitivity, -deflection)
    shift = add(mul(props['sensitivity'], scaled), aoa_bias)
    cl = mul(scaled, props['cl'][0 if deflection >= 0 else 1])
    cd = mul(shift, props['cd'][0 if shift >= 0 else 1])
    return [deflection, shift, cl, cd, mul(props['wing_aoa'], -deflection)]


def prepare_rows(knots):
    """Build prepared ascending rows from (x, values), rounding as float32."""
    data = [(f32(x), [f32(v) for v in values]) for x, values in knots]
    if any(b[0] <= a[0] for a, b in zip(data, data[1:])):
        raise ValueError('Knots must be strictly increasing after float rounding')
    return [(x, f32(1.0 / sub(data[i+1][0], x)) if i+1 < len(data) else 0.0, v)
            for i, (x, v) in enumerate(data)]


def density_at_height(height, rho0=f32(1.225), ceiling=f32(18300)):
    """Atmosphere helper 0x1019881d0; arguments/globals are float32."""
    h = min(height, ceiling)
    polynomial = f32(2.287190065873038e-19)
    for coefficient in [-5.83556030646186e-14, 3.5311800150594763e-09,
                        -9.593870345270261e-05, 1.0]:
        polynomial = add(mul(polynomial,h),f32(coefficient))
    return f32(mul(mul(ceiling,rho0),polynomial)/max(height,ceiling))


def selected_aircraft_properties(config):
    """Prepare explicitly configured F-16/JAS control blocks from the datamine.

    Loader-inspired adapter, not a full BLK loader port. Reject missing fields
    except the loader's documented default angle multiplier (1, 0, 1);
    preserve supplied 2D order, including its density conversion. Runtime table
    evaluation is independently checked by verify_control_mixer.py.
    """
    def table(prefix,width,xscale=1.0,numbered_only=False):
        knots=[]
        for i in range(10):
            key=prefix+str(i)
            if key in config:
                row=config[key]
                if isinstance(row[0],list):row=row[0]  # first BLK parameter of that name
                x,*v=row
                if len(v)!=width:raise ValueError(key)
                x=mul(f32(x),f32(xscale))
                if not knots or sub(x,knots[-1][0])>f32(4e-19):knots.append((x,v))
        if not knots and prefix in config and not numbered_only:
            value=config[prefix]
            if width==1 and isinstance(value,(int,float)):value=[value]
            if width==1 and len(value)==4:
                # Point4 fallback in 1019e35e0: two (x,y) knots.
                knots=[(mul(f32(value[0]),f32(xscale)),[value[1]])]
                if sub(f32(value[2]),f32(value[0]))>f32(4e-19):knots.append((mul(f32(value[2]),f32(xscale)),[value[3]]))
            elif len(value)==width:knots=[(0.,value)]
            else:raise ValueError(prefix)
        if not knots:
            if width==3 and not numbered_only:return prepare_rows([(0,[1,0,1])])
            if width==1 and not numbered_only:return prepare_rows([(0,[1.])])
            raise ValueError('Missing explicit table '+prefix)
        return prepare_rows(knots)
    one,two=[],[]
    for axis in ['Roll','Pitch','Yaw']:
        prefix='AnglesMultiplier'+axis
        one.append(table(prefix,3,f32(1/3.6)))
        family=[]
        for i in range(10):
            key=prefix+'2D'+str(i)
            if key in config:
                family.append((density_at_height(f32(config[key])),table(key,3,f32(1/3.6),True)))
        two.append(family)
    prepared=[]
    for family in two:
        if any(b[0]<=a[0] for a,b in zip(family,family[1:])):
            raise ValueError('2D density rows must be increasing')
        prepared.append([(x,f32(1/sub(family[i+1][0],x)) if i+1<len(family) else 0.,values)
                         for i,(x,values) in enumerate(family)])
    return dict(angles=[[f32(v) for v in config['Angles'+axis]] for axis in ['Roll','Pitch','Yaw']],
                tables_1d=one,tables_2d=prepared,sensitivity=f32(config['Sensitivity']),
                sensitivity_curve=table('SensitivityMultiplier',1),
                arcade_curve=table('ArcadeSensitivityMultiplier',1),
                cl=list(map(f32,config['SensitivityCl'])),cd=list(map(f32,config['SensitivityCd'])),
                wing_aoa=f32(config.get('SensitivityWingAoa',0.)))
