(() => {
  'use strict';
  const STEP=1/48,STATE_VERSION=1,STATE_SIZE=32;
  const mapping={KeyW:[1,-1],ArrowUp:[1,1],KeyS:[1,1],ArrowDown:[1,-1],KeyA:[2,-1],ArrowLeft:[2,-1],KeyD:[2,1],ArrowRight:[2,1],KeyQ:[0,-1],KeyE:[0,1]};
  const actions={Space:'pause',Escape:'stop',KeyC:'camera',KeyR:'again',Digit1:1,Numpad1:1,Digit2:.5,Numpad2:.5,Digit3:.25,Numpad3:.25};
  const editing=target=>!!target?.closest?.('input,select,textarea,[contenteditable]:not([contenteditable="false"])');
  class Keys {
    constructor(){this.held=new Set();this.latched=new Set();}
    down(event,active,canRestart=false){
      if(event.repeat||event.ctrlKey||event.altKey||event.metaKey||editing(event.target))return null;
      const code=event.code;
      if(code==='KeyR'?!canRestart:!active)return null;
      if(!(code in mapping)&&!(code in actions))return null;
      event.preventDefault?.();
      if(mapping[code]){this.held.add(code);this.latched.add(code);return 'input';}
      return actions[code];
    }
    up(event){this.held.delete(event.code);}
    clear(){this.held.clear();this.latched.clear();}
    take(){
      const positive=[false,false,false],negative=[false,false,false];
      for(const code of new Set([...this.held,...this.latched])){
        const pair=mapping[code];if(pair)(pair[1]>0?positive:negative)[pair[0]]=true;
      }
      this.latched.clear();return positive.map((p,i)=>Number(p)-Number(negative[i]));
    }
  }
  class Clock {
    constructor(){this.acc=0;this.last=null;this.paused=false;this.scale=1;this.steps=0;}
    pause(){this.paused=true;this.last=null;}
    resume(){this.paused=false;this.last=null;}
    frame(now,step){
      if(this.paused){this.last=null;return 0;}
      if(this.last===null){this.last=now;return this.acc/STEP;}
      this.acc+=Math.min(Math.max(0,(now-this.last)/1000),.1)*this.scale;this.last=now;
      let count=0;
      while(this.acc+1e-12>=STEP&&count<24){this.acc=Math.max(0,this.acc-STEP);++count;++this.steps;if(step()===false)break;}
      return Math.min(1,this.acc/STEP);
    }
  }
  function appendInput(log,az,el,roll){const last=log.at(-1);if(last&&last[1]===az&&last[2]===el&&last[3]===roll)last[0]++;else log.push([1,az,el,roll]);}
  function slerp(a,b,t){
    let dot=a.reduce((sum,v,i)=>sum+v*b[i],0);if(dot<0){b=b.map(v=>-v);dot=-dot;}
    dot=Math.min(1,dot);let q;
    if(dot>.9995)q=a.map((v,i)=>v+(b[i]-v)*t);
    else{const angle=Math.acos(dot),sin=Math.sin(angle);q=a.map((v,i)=>(v*Math.sin((1-t)*angle)+b[i]*Math.sin(t*angle))/sin);}
    const length=Math.hypot(...q);return q.map(v=>v/length);
  }
  function interpolate(a,b,t){
    const state=b.slice();for(let i=0;i<23;i++)state[i]=a[i]+(b[i]-a[i])*t;
    state.splice(7,4,...slerp(a.slice(7,11),b.slice(7,11),t));state.splice(19,4,...slerp(a.slice(19,23),b.slice(19,23),t));return state;
  }
  function prepareBody(submitted,rowKey,baseId,launcher){
    const {missiles,launch_aircraft,...shared}=submitted;
    return {...shared,missile:baseId(rowKey),launcher:launcher(shared.launcher,launch_aircraft[rowKey]),early_exit:false};
  }
  function handoff(entry,label){
    const doc=entry.document,launch=doc.scenario.launcher;
    if(entry.aircraft){launch.aircraft_id=entry.aircraft.id;launch.aircraft=entry.aircraft;}
    else{delete launch.aircraft_id;delete launch.aircraft;}
    doc.scenario.missile=entry.rowKey;doc.scenario.label=label;return doc;
  }
  function targetFrame(rows,time){
    let low=0,high=rows.length-1;
    while(low<high){const middle=Math.ceil((low+high)/2);if(rows[middle].time_s<=time)low=middle;else high=middle-1;}
    const a=rows[low],b=rows[Math.min(low+1,rows.length-1)],fraction=b.time_s>a.time_s?Math.max(0,Math.min(1,(time-a.time_s)/(b.time_s-a.time_s))):0;
    return {position:a.target.map((v,i)=>v+(b.target[i]-v)*fraction),quaternion:slerp(a.target_quaternion,b.target_quaternion,fraction)};
  }
  const api={STEP,STATE_VERSION,STATE_SIZE,Keys,Clock,appendInput,slerp,interpolate,prepareBody,handoff,targetFrame};
  if(typeof module!=='undefined')module.exports=api;else window.missileLiveCore=api;
})();
