"""Finite float32 ports of 0x10198c320/440/4d0, including instruction order.

sin/sincos are host libm rounded to float32, as in the differential harness.
This module does not prepare the runtime polar or claim target libm bit parity.
"""
import math
from component_assembly import f32,add,sub,mul


def sin(x):return f32(math.sin(x))
def div(a,b):return f32(a/b)


def calc_cl(p,a):
    a=f32(a)
    if p['aoaLineL']<=a<=p['aoaLineH']:
        return add(mul(mul(a,p['clLineCoeff']),p['cyMult']),p['cl0'])
    positive=add(a,f32(.01))>=p['aoaLineH'];s=1. if positive else -1.
    suffix='H' if positive else 'L'
    crit=p['aoaCrit'+suffix];cy=p['cyCrit'+suffix];after=p['clAfterCrit'+suffix]
    da=sub(a,crit)
    if mul(da,s)<=0.:
        x=sub(crit,a)
        return sub(cy,mul(mul(mul(x,x),s),p['parabCyCoeff'+suffix]))
    maxang=max(40.,p['maxDistAng']);sa=mul(s,a)
    if sa<=maxang:
        if sa<=p['maxDistAng']:
            pa=p['parabAngle']
            if mul(da,s)<pa:return sub(cy,mul(mul(mul(da,da),s),p['declineCoeff']))
            h=mul(sin(mul(f32(.039269909262657166),p['maxDistAng'])),after)
            x=sub(mul(sub(p['maxDistAng'],pa),s),crit);den=mul(x,x)
            coeff=div(sub(sub(cy,h),mul(mul(mul(pa,pa),s),p['declineCoeff'])),den) if den>f32(4e-19) else 0.
            x=sub(mul(p['maxDistAng'],s),a)
            return add(mul(mul(x,x),coeff),h)
        return mul(after,sin(mul(mul(a,f32(.039269909262657166)),s)))
    if sa<=140.:
        local_sign=-s if sa>90. else s
        local_angle=sub(180.,sa) if sa>90. else sa
        one=sin(mul(f32(.039269909262657166),maxang))
        two=sin(sub(f32(-1.5707963705062866),mul(max(0.,add(maxang,-40.)),f32(.03141592815518379))))
        correction=add(two,one)
        correction=mul(add(mul(add(mul(sa,f32(-.01)),f32(.3999999761581421)),correction),correction),s)
        wave=mul(sin(add(mul(local_angle,f32(.03141592815518379)),f32(.3141592741012573))),local_sign)
        return mul(add(wave,correction),mul(after,s))
    return mul(mul(mul(s,s),after),sin(add(mul(sa,f32(.039269909262657166)),f32(-7.0685834884643555))))


def calc_cd(p,a):
    a=f32(a);line=add(mul(p['clLineCoeff'],a),p['cl0'])
    delta=sub(a,p['aoaCritH' if a>=0. else 'aoaCritL'])
    if a<0.:delta=-delta
    cd=add(add(mul(mul(line,line),p['indCoeff']),p['cd0']),mul(delta,p['cdAfterCoeff']) if delta>=0. else 0.)
    bound=add(mul(abs(sin(mul(a,f32(.01745329238474369)))),p['cyCritH']),f32(.15))
    return min(cd,bound)


def calc_c(p,a,angle,cl_add=0.,cd_coeff=1.):
    cd=mul(calc_cd(p,a),f32(cd_coeff));cl=add(calc_cl(p,a),f32(cl_add))
    radians=mul(f32(angle),f32(.01745329238474369));sn=sin(radians);cs=f32(math.cos(radians))
    return [mul(sub(mul(cs,cd),mul(cl,sn)),p['kq']),mul(add(mul(cs,cl),mul(cd,sn)),p['clKq'])]
