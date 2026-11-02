// Target steering, expression-for-expression with missile_live_motion.py.
#pragma once
#include <vector>

namespace typed {
inline double steering_dot(const d3& a,const d3& b) { return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; }
inline d3 steering_cross(const d3& a,const d3& b) { return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]}; }
inline d3 steering_unit(d3 a) { double length=std::sqrt(steering_dot(a,a));for(double& x:a)x=x/length;return a; }
inline d4 frame_quaternion(const d3& f,const d3& u,const d3& r) {
    double t=f[0]+u[1]+r[2];d4 q;
    if(t>0){double s=2.*std::sqrt(t+1.);q={(u[2]-r[1])/s,(r[0]-f[2])/s,(f[1]-u[0])/s,s/4.};}
    else if(f[0]>u[1]&&f[0]>r[2]){double s=2.*std::sqrt(1.+f[0]-u[1]-r[2]);q={s/4.,(u[0]+f[1])/s,(r[0]+f[2])/s,(u[2]-r[1])/s};}
    else if(u[1]>r[2]){double s=2.*std::sqrt(1.+u[1]-f[0]-r[2]);q={(u[0]+f[1])/s,s/4.,(r[1]+u[2])/s,(r[0]-f[2])/s};}
    else{double s=2.*std::sqrt(1.+r[2]-f[0]-u[1]);q={(r[0]+f[2])/s,(r[1]+u[2])/s,s/4.,(f[1]-u[0])/s};}
    return f32s(q);
}
struct SteeredPath {
    struct Segment {bool straight;double t0,t1;d3 p0,p1,velocity;d4 quaternion;bool dynamic=false;d3 velocity0{};d4 quaternion0{};};
    double V=0,A=0,s=0,c=0,k1=0,k2=0,rs=0,rc=0,diag=0,last_time=0;
    d3 position{},velocity{},p{},f{},u{},r{};d4 quaternion{};
    std::vector<Segment> segments;
    void advance(double old,double now,int az,int el,int roll) {
        if(!az&&!el&&!roll){
            if(!segments.empty()&&segments.back().straight)segments.back().t1=now;
            else{Segment seg{};seg.straight=true;seg.t0=old;seg.t1=now;seg.p0=old==0?position:p;
                for(int i=0;i<3;++i)seg.velocity[i]=old==0?velocity[i]:V*f[i];
                seg.quaternion=old==0?quaternion:frame_quaternion(f,u,r);segments.push_back(seg);}
        }else{
            if(!segments.empty()&&segments.back().straight){const auto& last=segments.back();for(int i=0;i<3;++i)p[i]=last.p0[i]+last.velocity[i]*(last.t1-last.t0);}
            if(roll){d3 old_u=u,old_r=r;for(int i=0;i<3;++i){u[i]=rc*old_u[i]+roll*rs*old_r[i];r[i]=rc*old_r[i]-roll*rs*old_u[i];}}
            double travel=V*(now-old);d3 p1,f1,u1;
            if(az||el){double scale=az&&el?diag:1.,a=az*scale,e=el*scale;d3 n,n1;
                for(int i=0;i<3;++i)n[i]=e*u[i]+a*r[i];
                for(int i=0;i<3;++i){p1[i]=p[i]+travel*(k1*f[i]+k2*n[i]);f1[i]=c*f[i]+s*n[i];n1[i]=c*n[i]-s*f[i];u1[i]=u[i]+e*(n1[i]-n[i]);}
            }else{for(int i=0;i<3;++i)p1[i]=p[i]+travel*f[i];f1=f;u1=u;}
            f=steering_unit(f1);double projection=steering_dot(u1,f);
            for(int i=0;i<3;++i)u[i]=u1[i]-projection*f[i];u=steering_unit(u);r=steering_cross(f,u);
            Segment seg{};seg.t0=old;seg.t1=now;seg.p0=p;seg.p1=p1;
            for(int i=0;i<3;++i)seg.velocity[i]=V*f[i];seg.quaternion=frame_quaternion(f,u,r);
            segments.push_back(seg);p=p1;
        }
        last_time=now;
    }
    TargetPoint at(double time) const {
        if(time>last_time)bail("steered target queried ahead of its inputs");
        TargetPoint result;
        if(time<=0){for(int i=0;i<3;++i){result.position[i]=f32(position[i]+velocity[i]*(time-0.));result.velocity[i]=f32(velocity[i]);}result.quaternion=f32s(quaternion);return result;}
        size_t low=0,high=segments.size()-1;
        while(low<high){size_t mid=(low+high)/2;if(time<=segments[mid].t1)high=mid;else low=mid+1;}
        const auto& seg=segments[low];double fraction=(time-seg.t0)/(seg.t1-seg.t0);
        for(int i=0;i<3;++i){result.position[i]=f32(seg.straight?seg.p0[i]+seg.velocity[i]*(time-seg.t0):seg.p0[i]+(seg.p1[i]-seg.p0[i])*fraction);result.velocity[i]=f32(seg.velocity[i]);}
        result.quaternion=f32s(seg.quaternion);
        if(seg.dynamic){
            double sign=0;for(int i=0;i<4;++i)sign+=seg.quaternion0[i]*seg.quaternion[i];sign=sign<0?-1.:1.;
            d4 q{};double square=0.;for(int i=0;i<4;++i){q[i]=seg.quaternion0[i]+(sign*seg.quaternion[i]-seg.quaternion0[i])*fraction;square+=q[i]*q[i];}
            double length=std::sqrt(square);for(int i=0;i<4;++i)result.quaternion[i]=f32(q[i]/length);
            for(int i=0;i<3;++i)result.velocity[i]=f32(seg.velocity0[i]+(seg.velocity[i]-seg.velocity0[i])*fraction);
        }
        return result;
    }
};
}
