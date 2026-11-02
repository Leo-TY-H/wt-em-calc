"""Bounded live acceptance corpus. Retains scalar counters and replayable failures."""
import argparse
from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import random
import subprocess
import sys
import threading
import time
from benchmark_output import OutputBudget

ROOT=Path(__file__).resolve().parents[1]
MISSILES=['us_aim9l_sidewinder','us_aim_120c_5','us_aim9b_sidewinder','us_aim9m_sidewinder',
         'us_aim7f_sparrow','us_aim7m_sparrow','su_r_73','su_r_60','fr_r_550_magic_2','fr_mica_em',
         'uk_sraam','us_aim_54c']


def canonical_equal(a,b):
    return json.dumps(a,sort_keys=True)==json.dumps(b,sort_keys=True)


class Engine:
    def __init__(self,command,env=None,wrapped=True):
        self.process=subprocess.Popen(command,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,
                                      text=True,encoding='utf-8',env=env,bufsize=1,
                                      creationflags=subprocess.CREATE_NO_WINDOW if os.name=='nt' else 0)
        self.diagnostics=[];self.wrapped=wrapped;self.compare_count=0;self.compare_failures=0
        def drain():
            for line in self.process.stderr:
                if line.startswith('{'):
                    try:
                        report=json.loads(line)
                        if report.get('type')=='typed_simulate_compare':
                            self.compare_count+=1
                            if not report.get('result_equal'):self.compare_failures+=1
                    except ValueError:pass
                if len(self.diagnostics)<100:self.diagnostics.append(line[:2000])
        self.thread=threading.Thread(target=drain,daemon=True);self.thread.start()
    def request(self,request):
        self.process.stdin.write(json.dumps(request,separators=(',',':'))+'\n');self.process.stdin.flush()
        while True:
            line=self.process.stdout.readline()
            if not line:raise RuntimeError('Engine ended: '+''.join(self.diagnostics[-3:]))
            value=json.loads(line)
            if 'error' in value:raise RuntimeError(str(value['error']))
            if not self.wrapped:return value
            if value.get('type')=='result':return value['result']
    def close(self):
        self.process.stdin.close();self.process.wait(timeout=30);self.thread.join(timeout=5)
        self.process.stdout.close();self.process.stderr.close()


def logs():
    rng=random.Random(20261102)
    result=[]
    patterns=[[[240,0,1,0]],[[240,0,-1,0]],[[96,0,0,1],[144,1,0,0]],[[240,1,1,0]],
              [[48,0,0,0],[96,-1,1,-1],[96,0,0,0]],[[240,0,0,-1]],[[2600,0,1,0]]]
    for i in range(60):
        log=deepcopy(patterns[i]) if i<len(patterns) else [[rng.randint(1,24),rng.choice([-1,0,1]),rng.choice([-1,0,1]),rng.choice([-1,0,1])] for _ in range(12)]
        if i%11==0:log=[[1,(-1)**j,(-1)**(j//2),(-1)**(j//3)] for j in range(240)]
        result.append(log)
    return result


def scenario(missile,case,log):
    speed=[250.,5.,2000.][case%3];pitch=[0.,90.,-90.,30.][case%4];radians=math.radians(pitch)
    target=dict(position=[12000.,5000.,0.],velocity=[speed*math.cos(radians),speed*math.sin(radians),0.],angles=[0.,pitch,0.])
    if case==6:target.update(velocity=[250.,0.,0.],angles=[0.,0.,0.])
    duration=min(300.,max(.1,sum(row[0] for row in log)/48+.02))
    control=dict(load_g=9. if case==6 else [1.5,9.,15.][case%3],inputs=log)
    if case%5==0:control['stop_after_steps']=min(37,math.ceil(duration/(1/48)))
    return dict(missile=missile,launcher=dict(position=[0.,5000.,0.],velocity=[300.,0.,0.],angles=[0.,0.,0.]),target=target,duration=duration,early_exit=False,target_control=control)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--cases',type=int,default=60);parser.add_argument('--missiles',type=int,default=12)
    parser.add_argument('--linux-native',action='store_true');parser.add_argument('--report',default='corpus.json');parser.add_argument('--reference',action='store_true');parser.add_argument('--wasm',action='store_true');parser.add_argument('--baseline',type=Path,default=ROOT.parent/'WT-FM-app')
    args=parser.parse_args();out=ROOT/'outputs/live-missile';budget=OutputBudget(out,'live-corpus')
    started=time.perf_counter();exe=ROOT/'.native_missile_engine/missile-engine.exe'
    env=dict(os.environ,WT_MISSILE_TYPED='1' if args.wasm else 'compare',WT_MISSILE_TYPED_LOG='1')
    linux_root='/mnt/'+ROOT.drive[0].lower()+ROOT.as_posix()[2:] if args.linux_native else ''
    command=['wsl','-d','Ubuntu','--',linux_root+'/.linux_missile_engine/missile-engine','--root',linux_root] if args.linux_native else [str(exe),'--root',str(ROOT)]
    native=Engine([*command,'--mode','simulator','--batch'],env)
    live=Engine([*command,'--mode','live'],wrapped=False)
    baseline=Engine([str(args.baseline/'.native_missile_engine/missile-engine.exe'),'--root',str(ROOT),'--mode','simulator','--batch'])
    wasm=Engine(['node',str(ROOT/'scripts/missile_wasm/run_node.mjs'),str(ROOT)],wrapped=False) if args.wasm else None
    if args.reference:
        os.environ['WT_MISSILE_ENGINE']='python';os.environ['WT_MISSILE_BACKEND']='reference'
        from missile_worker import simulate
    result=dict(command=sys.argv,source_root=str(ROOT),cases=0,steps=0,baseline_equal=0,zero_equal=0,reference_equal=0,live_equal=0,wasm_equal=0,wasm_outcomes_equal=0,wasm_q0_equal=0,wasm_differences=0,max_position_difference_m=0.,failure_count=0,failures=[],input_sha256=hashlib.sha256(json.dumps(logs()).encode()).hexdigest())
    def check(equal,name,request,extra=None):
        if equal:result[name]+=1
        elif name=='wasm_equal':result['wasm_differences']+=1
        else:
            failure=dict(check=name,request=request,detail=extra);result['failure_count']+=1
            if len(result['failures'])<10:result['failures'].append(failure)
    try:
        index=json.loads((ROOT/'scripts/missile_model/launch-profiles/index.json').read_bytes())
        available={p['path'].removesuffix('.json') for p in index['profiles']}
        missiles=[m for m in MISSILES if m in available]
        if len(missiles)<args.missiles:raise RuntimeError('Acceptance missile list has missing profiles: '+str(set(MISSILES)-available))
        for missile in missiles[:args.missiles]:
            for case,log in enumerate(logs()[:args.cases]):
                req=scenario(missile,case,log);doc=native.request(req);result['cases']+=1
                if args.reference:check(canonical_equal(simulate(req),doc),'reference_equal',req)
                if case==0 and not args.linux_native:
                    plain=deepcopy(req);del plain['target_control'];before=baseline.request(plain);after=native.request(plain)
                    check(canonical_equal(before,after),'baseline_equal',plain)
                    zero=deepcopy(plain);zero['target_control']=dict(load_g=9.,inputs=[]);zero_doc=native.request(zero)
                    del zero_doc['scenario']['target_control'];zero_doc['model']['target_motion']='constant_velocity'
                    for row in zero_doc['trajectory']+[zero_doc['final_body_sample']]:row.pop('target_quaternion')
                    check(canonical_equal(after,zero_doc),'zero_equal',zero)
                for name,engine in [('live_equal',live),('wasm_equal',wasm)]:
                    if engine is None:continue
                    start=deepcopy(req);start['target_control']=dict(load_g=req['target_control']['load_g'],inputs=[])
                    handle=engine.request({'start':start})['handle'];steps=0;terminal=False
                    try:
                        for count,az,el,roll in log+[[14400,0,0,0]]:
                            for _ in range(count):
                                steps+=1;state=engine.request({'step':[handle,az,el,roll,int(steps==req['target_control'].get('stop_after_steps'))]})
                                if state['status']==1:terminal=True;break
                            if terminal:break
                        actual=engine.request({'result':handle});check(canonical_equal(actual,doc),name,req)
                        if name=='live_equal':result['steps']+=steps
                        else:
                            check(actual['outcome']['code']==doc['outcome']['code'],'wasm_outcomes_equal',req)
                            if actual['trajectory'][0]['target_quaternion']==doc['trajectory'][0]['target_quaternion']:result['wasm_q0_equal']+=1
                            if actual['outcome']['code']!=doc['outcome']['code']:raise RuntimeError('WASM/native outcome disagreement; stop for review')
                            for a,b in zip(actual['trajectory'],doc['trajectory']):
                                result['max_position_difference_m']=max(result['max_position_difference_m'],math.dist(a['missile'],b['missile']))
                    finally:engine.request({'free':handle})
                print(json.dumps(dict(cases=result['cases'],failures=len(result['failures']),missile=missile,case=case)),flush=True)
        # Compare diagnostics are mandatory: fallback cannot masquerade as acceptance.
        reports=[]
        for line in native.diagnostics:
            try:reports.append(json.loads(line))
            except ValueError:pass
        result['compare_reports_retained']=len(reports)
        result['compare_count']=native.compare_count
        result['compare_failed_count']=native.compare_failures
        result['compare_failures']=[r for r in reports if r.get('type')=='typed_simulate_compare' and not r.get('result_equal')]
    finally:
        for engine in [native,live,baseline,wasm]:
            if engine:engine.close()
        result['elapsed_s']=time.perf_counter()-started
        budget.json(out/args.report,result);budget.finish()
    if result['failure_count'] or result['compare_failed_count']:raise SystemExit(1)


if __name__=='__main__':main()
