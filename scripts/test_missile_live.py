"""Geometry, validation and reference replay checks for interactive game targets."""
import math
import json
import os
os.environ['WT_MISSILE_ENGINE']='python'
os.environ['WT_MISSILE_BACKEND']='reference'
import random
import struct
import unittest
from copy import deepcopy
from missile_worker import quaternion,validate,Simulation,simulate,STEP,add
from missile_live_motion import steered_motion,steered_advance,steered_point,frame_quaternion,series_sin_cos,dot,cross


def request():
    return dict(missile='us_aim9l_sidewinder',duration=.25,
                launcher=dict(position=[0,5000,0],velocity=[300,0,0],angles=[0,0,0]),
                target=dict(position=[4000,5000,0],velocity=[250,0,0],angles=[0,0,0]))


class Steering(unittest.TestCase):
    def motion(self,speed=250,load=9):
        target=request()['target'];target['velocity']=[speed,0,0]
        return steered_motion(target,dict(load_g=load))

    def test_series(self):
        for i in range(10001):
            x=-.5+i/10000;s,c=series_sin_cos(x)
            self.assertLessEqual(abs(s-math.sin(x)),4e-16);self.assertLessEqual(abs(c-math.cos(x)),4e-16)

    def test_quaternion_and_vertical_frame(self):
        rng=random.Random(3107)
        for i in range(2000):
            q=quaternion([rng.uniform(-360,360),90 if i%10==0 else -90 if i%10==1 else rng.uniform(-90,90),rng.uniform(-360,360)])
            x,y,z,w=q
            f=[1-2*(y*y+z*z),2*(x*y+w*z),2*(x*z-w*y)]
            u=[2*(x*y-w*z),1-2*(x*x+z*z),2*(y*z+w*x)]
            r=[2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y)]
            result=frame_quaternion(f,u,r)
            self.assertLess(min(max(abs(a-b) for a,b in zip(q,result)),max(abs(a+b) for a,b in zip(q,result))),1e-6)
        target=request()['target'];target.update(velocity=[0,250,0],angles=[0,90,0])
        m=steered_motion(target,dict(load_g=9));self.assertLess(max(abs(a-b) for a,b in zip(m['u0'],[-1,0,0])),1e-6)

    def test_frames_and_continuity(self):
        rng=random.Random(171)
        for case in range(60):
            m=self.motion([5,250,2000][case%3],[1.5,9,15][case%3]);t=0.
            for i in range(240):
                old=steered_point(m,t)['position'];inputs=tuple(rng.choice([-1,0,1]) for _ in range(3));new=add(t,STEP)
                steered_advance(m,t,new,*inputs)
                self.assertEqual(steered_point(m,t)['position'],old)
                self.assertLess(abs(dot(m['f'],m['f'])-1),1e-12);self.assertLess(abs(dot(m['u'],m['u'])-1),1e-12)
                self.assertLess(abs(dot(m['f'],m['u'])),1e-12);self.assertEqual(cross(m['f'],m['u']),m['r']);t=new
            with self.assertRaisesRegex(ValueError,'ahead'):steered_point(m,t+STEP)

    def test_loop_roll_and_right(self):
        m=self.motion();period=2*math.pi*m['V']/m['A'];t=0.;points=[m['position']]
        for i in range(round(period/STEP)):
            new=add(t,STEP);steered_advance(m,t,new,0,1,0);t=new;points.append(m['p'])
        self.assertLess(math.dist(points[0],points[-1]),m['V']*STEP)
        self.assertLess(abs(max(p[1] for p in points)-min(p[1] for p in points)-2*m['V']**2/m['A']),.05)
        m=self.motion();t=0.
        for i in range(24):
            new=add(t,STEP);steered_advance(m,t,new,0,0,1);t=new
        self.assertLess(max(abs(a-b) for a,b in zip(frame_quaternion(m['f'],m['u'],m['r']),quaternion([0,0,90]))),1e-6)
        self.assertEqual(m['p'][2],0.);m=self.motion();steered_advance(m,0,STEP,1,0,0);self.assertGreater(m['p'][2],0)
        m=self.motion();t=0.
        for _ in range(round(period/STEP/2)):
            new=add(t,STEP);steered_advance(m,t,new,0,1,0);t=new
        self.assertLess(m['f'][0],-.99);self.assertLess(m['u'][1],-.99)
        previous=m['p'][2];steered_advance(m,t,add(t,STEP),1,0,0);self.assertGreater(m['p'][2],previous)
        m=self.motion();t=0.
        for _ in range(10000):
            new=add(t,STEP);steered_advance(m,t,new,0,0,0);t=new
        heights=[m['position'][1]];drift=0.
        for _ in range(round(period/STEP)):
            new=add(t,STEP);drift=max(drift,abs((new-t)/STEP-1));steered_advance(m,t,new,0,1,0);t=new;heights.append(m['p'][1])
        diameter=2*m['V']**2/m['A']
        self.assertLess(abs(max(heights)-min(heights)-diameter),diameter*drift+.01)

    def test_corpus_geometry(self):
        from verify_missile_live import logs,scenario
        from kernels import f32
        steps=0
        for case,log in enumerate(logs()):
            req=scenario('us_aim9l_sidewinder',case,log)
            m=steered_motion(req['target'],req['target_control']);t=0.
            for count,az,el,roll in log:
                for _ in range(count):
                    earlier=steered_point(m,t)['position'];old_kind=m['segments'][-1]['kind'] if m['segments'] else 'constant_velocity'
                    new=add(t,STEP);steered_advance(m,t,new,az,el,roll)
                    seg=m['segments'][-1]
                    if seg['t0']==t:
                        later=[f32(x) for x in seg.get('p0',seg.get('position'))]
                        for a,b in zip(earlier,later):
                            bits=struct.unpack('I',struct.pack('f',abs(b)))[0]
                            ulp=struct.unpack('f',struct.pack('I',bits+1))[0]-abs(b)
                            self.assertLessEqual(abs(a-b),0. if old_kind=='constant_velocity' else ulp)
                    self.assertLess(abs(math.sqrt(dot(m['f'],m['f']))-1),1e-12)
                    self.assertLess(abs(math.sqrt(dot(m['u'],m['u']))-1),1e-12)
                    self.assertLess(abs(dot(m['f'],m['u'])),1e-12)
                    self.assertEqual(cross(m['f'],m['u']),m['r']);t=new;steps+=1
            with self.assertRaisesRegex(ValueError,'ahead'):steered_point(m,t+STEP)
        print('Corpus geometry:',steps,'steps, 60 logs',flush=True)

    def test_validation(self):
        req=request();req['target_control']={'load_g':9};self.assertFalse(validate(req)['early_exit'])
        for bad in [True,0,1.1,100000]:
            req['target_control']['stop_after_steps']=bad
            with self.assertRaises(ValueError):validate(req)
        req['target_control']={'load_g':9,'inputs':[[True,0,0,0]]}
        with self.assertRaises(ValueError):validate(req)

    def test_zero_input_and_stop_replay(self):
        req=request();req['early_exit']=False;baseline=simulate(req)
        controlled=deepcopy(req);controlled['target_control']={'load_g':9};result=simulate(controlled)
        stripped=json.loads(json.dumps(result));del stripped['scenario']['target_control'];stripped['model']['target_motion']='constant_velocity'
        for row in stripped['trajectory']+[stripped['final_body_sample']]:self.assertEqual(row.pop('target_quaternion'),quaternion(req['target']['angles']))
        self.assertEqual(stripped,baseline)
        run=Simulation(controlled)
        for i in range(3):run.step(1,1,-1,i==2)
        doc=run.document();self.assertEqual(doc['outcome']['code'],'stopped');self.assertEqual(doc['scenario']['target_control']['stop_after_steps'],3)
        self.assertEqual(simulate(doc['scenario']),doc)


if __name__=='__main__':unittest.main()
