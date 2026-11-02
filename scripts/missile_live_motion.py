"""Deterministic, constant-speed attitude steering for the game simulator.

Only the target changes direction. Missile dynamics remain in the existing engine.
The functions use the translated runtime's arithmetic subset.
"""
import math
from kernels import f32
from flight_session import linear_point
from frame_session import STEP


def dot(a, b):
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]


def cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]


def unit(a):
    length = math.sqrt(dot(a, a))
    return [x / length for x in a]


def series_sin_cos(x):
    z = x*x
    sine = x*(1.+z*(-1./6.+z*(1./120.+z*(-1./5040.+z*(1./362880.+z*(-1./39916800.+z*(1./6227020800.+z*(-1./1307674368000.+z/355687428096000.))))))))
    cosine = 1.+z*(-1./2.+z*(1./24.+z*(-1./720.+z*(1./40320.+z*(-1./3628800.+z*(1./479001600.+z*(-1./87178291200.+z/20922789888000.)))))))
    return sine, cosine


def frame_quaternion(f, u, r):
    trace = f[0] + u[1] + r[2]
    if trace > 0:
        s = 2.*math.sqrt(trace+1.)
        q = [(u[2]-r[1])/s, (r[0]-f[2])/s, (f[1]-u[0])/s, s/4.]
    elif f[0] > u[1] and f[0] > r[2]:
        s = 2.*math.sqrt(1.+f[0]-u[1]-r[2])
        q = [s/4., (u[0]+f[1])/s, (r[0]+f[2])/s, (u[2]-r[1])/s]
    elif u[1] > r[2]:
        s = 2.*math.sqrt(1.+u[1]-f[0]-r[2])
        q = [(u[0]+f[1])/s, s/4., (r[1]+u[2])/s, (r[0]-f[2])/s]
    else:
        s = 2.*math.sqrt(1.+r[2]-f[0]-u[1])
        q = [(r[0]+f[2])/s, (r[1]+u[2])/s, s/4., (f[1]-u[0])/s]
    return [f32(x) for x in q]


def steered_motion(target, control):
    from missile_worker import quaternion, ESCAPE_GRAVITY
    v = target['velocity']
    speed = math.sqrt(dot(v, v))
    if speed < 5.-1e-9:
        raise ValueError('A steered target must fly at least 18 km/h')
    load = control['load_g']
    acceleration = ESCAPE_GRAVITY*math.sqrt(load*load-1.)
    angle = acceleration*STEP/speed
    sh, ch = series_sin_cos(angle/2.)
    rh, rch = series_sin_cos(math.radians(180.)*STEP/2.)
    q = quaternion(target['angles'])
    x, y, z, w = q
    up = [2.*(x*y-w*z), 1.-2.*(x*x+z*z), 2.*(y*z+w*x)]
    right = [2.*(x*z+w*y), 2.*(y*z-w*x), 1.-2.*(x*x+y*y)]
    f = [a/speed for a in v]
    projection = dot(up, f)
    u = [up[i]-projection*f[i] for i in range(3)]
    if math.sqrt(dot(u,u)) >= 1e-3:
        u = unit(u)
        r = cross(f,u)
    else:
        projection = dot(right,f)
        r = unit([right[i]-projection*f[i] for i in range(3)])
        u = cross(r,f)
    return dict(kind='steered', position=target['position'], velocity=v, quaternion=q,
                V=speed, A=acceleration, s=2.*sh*ch, c=1.-2.*sh*sh,
                k1=2.*sh*ch/angle, k2=2.*sh*sh/angle, rs=2.*rh*rch,
                rc=1.-2.*rh*rh, diag=math.sqrt(.5), f0=f[:], u0=u[:], r0=r[:],
                p=target['position'][:], f=f, u=u, r=r, segments=[], last_time=0.)


def steered_advance(m, old, new, az, el, roll):
    segments = m['segments']
    p, f, u, r = m['p'], m['f'], m['u'], m['r']
    if az == 0 and el == 0 and roll == 0:
        if segments and segments[-1]['kind'] == 'constant_velocity':
            segments[-1]['t1'] = new
        else:
            initial = old == 0.
            segments.append(dict(kind='constant_velocity', t0=old, t1=new, epoch=old,
                                 position=m['position'] if initial else p,
                                 velocity=m['velocity'] if initial else [m['V']*x for x in f],
                                 quaternion=m['quaternion'] if initial else frame_quaternion(f,u,r)))
    else:
        if segments and segments[-1]['kind'] == 'constant_velocity':
            last = segments[-1]
            p = [last['position'][i]+last['velocity'][i]*(last['t1']-last['t0']) for i in range(3)]
        if roll:
            u, r = ([m['rc']*u[i]+roll*m['rs']*r[i] for i in range(3)],
                    [m['rc']*r[i]-roll*m['rs']*u[i] for i in range(3)])
        travel = m['V']*(new-old)
        if az or el:
            scale = m['diag'] if az and el else 1.
            a, e = az*scale, el*scale
            n = [e*u[i]+a*r[i] for i in range(3)]
            p1 = [p[i]+travel*(m['k1']*f[i]+m['k2']*n[i]) for i in range(3)]
            f1 = [m['c']*f[i]+m['s']*n[i] for i in range(3)]
            n1 = [m['c']*n[i]-m['s']*f[i] for i in range(3)]
            u1 = [u[i]+e*(n1[i]-n[i]) for i in range(3)]
        else:
            p1 = [p[i]+travel*f[i] for i in range(3)]
            f1, u1 = f, u
        f = unit(f1)
        projection = dot(u1,f)
        u = unit([u1[i]-projection*f[i] for i in range(3)])
        r = cross(f,u)
        segments.append(dict(kind='turn', t0=old, t1=new, p0=p, p1=p1,
                             v1=[m['V']*x for x in f], q1=frame_quaternion(f,u,r)))
        m['p'], m['f'], m['u'], m['r'] = p1, f, u, r
    m['last_time'] = new


def steered_point(m, time):
    if time > m['last_time']:
        raise ValueError('steered target queried ahead of its inputs')
    if time <= 0:
        return linear_point(dict(kind='constant_velocity', position=m['position'], velocity=m['velocity'], quaternion=m['quaternion']), time)
    segments = m['segments']
    low, high = 0, len(segments)-1
    while low < high:
        middle = (low+high)//2
        if time <= segments[middle]['t1']: high = middle
        else: low = middle+1
    segment = segments[low]
    if segment['kind'] == 'constant_velocity':
        return linear_point(segment,time)
    fraction = (time-segment['t0'])/(segment['t1']-segment['t0'])
    return dict(position=[f32(segment['p0'][i]+(segment['p1'][i]-segment['p0'][i])*fraction) for i in range(3)],
                velocity=[f32(x) for x in segment['v1']], quaternion=segment['q1'][:])


def append_input(log, az, el, roll):
    if log and log[-1][1:] == [az, el, roll]: log[-1][0] += 1
    else: log.append([1, az, el, roll])
