"""Diagnostic only: exact full-aircraft turns with a specified nonzero sideslip.

The production diagram fixes sideslip to zero. This probe changes that flight
condition, not a force or moment equation. It is not connected to the UI.
"""
import inspect,json,math,textwrap,time
from pathlib import Path
import numpy as np
import em_solver as em


def multiply(a,b):
    x,y,z,w=a;u,v,t,s=b
    return [w*u+x*s+y*t-z*v,w*v+y*s+z*u-x*t,w*t+z*s+x*v-y*u,w*s-x*u-y*v-z*t]


def slip_geometry(alpha_deg,bank_deg,speed,load,dt,ias_u,beta_deg):
    if beta_deg==0.:
        g=em.turn_geometry(alpha_deg,bank_deg,speed,load,dt,ias_u)
        return dict(g,side=np.array([0.,0.,1.]))
    alpha,bank,beta=map(math.radians,[alpha_deg,bank_deg,beta_deg])
    sa,ca,sb,cb,ss,cs=math.sin(alpha),math.cos(alpha),math.sin(bank),math.cos(bank),math.sin(beta),math.cos(beta)
    forward=np.array([ca*cs,-sa*cs,ss]);normal=np.array([sa,ca,0.]);side=np.array([-ca*ss,sa*ss,cs])
    up=cb*normal-sb*side;lateral=sb*normal+cb*side
    rate=float(em.G)*math.sqrt(max(0.,load*load-1.))/speed
    x,y,z=-up*math.sin(rate*dt*.5);w=math.cos(rate*dt*.5)
    r00=1-2*(y*y+z*z);r10=2*(x*y+w*z);r20=2*(x*z-w*y);r11=1-2*(x*x+z*z);r12=2*(y*z-w*x)
    yaw=math.atan2(-r20,r00);pitch=math.asin(max(-1.,min(1.,r10)));roll=math.atan2(-r12,r11)
    omega=-np.array([roll,yaw,pitch])/dt*(2/(1+max(1-float(em.f32(ias_u))*1e-5,.8)))
    q=multiply(multiply([math.sin(bank/2),0.,0.,math.cos(bank/2)],[0.,math.sin(beta/2),0.,math.cos(beta/2)]),[0.,0.,math.sin(alpha/2),math.cos(alpha/2)])
    return dict(forward=forward,normal=normal,side=side,up=up,lateral=lateral,omega=omega,quaternion=list(map(em.f32,q)),turn_rate=rate)


# Production now has explicit attitude support; retain this diagnostic's
# original beta property interface without source rewriting.
class SlipSolver(em.TrimSolver):
    @property
    def beta(self):return self.sideslip_attitude_deg
    @beta.setter
    def beta(self,value):self.sideslip_attitude_deg=float(value)


def main():
    fixtures=json.loads(Path('analysis/equilibrium-gap-recovery/fixtures.json').read_text())['fixtures']
    rows=[];start=time.monotonic()
    for index in [0,3,4,5,6,7,8,11]:
        f=fixtures[index];s=SlipSolver(f['aircraft'],f['settings']);p=f['bad'];best=None;trials=[]
        for beta in [.001,-.001,.003,-.003,.01,-.01,.03,-.03,.1,-.1,.3,-.3,1.,-1.,3.,-3.,8.,-8.]:
            s.beta=beta
            q=s.solve(p['speed_kmh'],p['load_g'],p['solution'],detailed=True,exhaustive=False)
            trials.append(dict(beta_input=beta,valid=q['valid'],reasons=q['reasons'],force_error_g=q['force_error_g'],angular_error_rad_s2=q['angular_error_rad_s2']))
            if q['valid']:
                best={k:v for k,v in q.items() if not k.startswith('_')};best['beta_input']=beta;best['sideslip_deg']=q['_detail']['result']['air']['beta'];break
        rows.append(dict(index=index,aircraft=s.name,speed=p['speed_kmh'],load=p['load_g'],solution=best,trials=trials))
        Path('analysis/gap-resolution/sideslip-probe.json').write_text(json.dumps(dict(rows=rows,seconds=time.monotonic()-start),indent=2)+'\n')
        print(index,s.name,'solved',bool(best),'beta',best['sideslip_deg'] if best else None,flush=True)

if __name__=='__main__':main()
