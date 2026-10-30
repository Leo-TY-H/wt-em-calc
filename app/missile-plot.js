(() => {
  'use strict';
  // Track gesture boundaries for final camera commits and paused-frame sizing.
  // Playback can continue by committing the live camera before each data update.
  function bindGestures(node, settled, host=window) {
    let active=false, dragging=false, wheelTimer=null, frame=null;
    const listeners=[];
    const listen=(target,name,fn,options)=>{target.addEventListener(name,fn,options);listeners.push(()=>target.removeEventListener(name,fn,options));};
    const cancelFrame=()=>{if(frame!==null)host.cancelAnimationFrame(frame);frame=null;};
    const finish=()=>{
      cancelFrame();
      if(dragging||wheelTimer!==null)return;
      frame=host.requestAnimationFrame(()=>{frame=null;if(!dragging&&wheelTimer===null&&active){active=false;settled();}});
    };
    const down=e=>{
      if(e.target.closest?.('.modebar')||e.button>2)return;
      cancelFrame();dragging=true;active=true;
    };
    const up=()=>{if(!dragging)return;dragging=false;finish();};
    const wheel=e=>{
      if(e.target.closest?.('.modebar'))return;
      active=true;cancelFrame();if(wheelTimer!==null)host.clearTimeout(wheelTimer);
      wheelTimer=host.setTimeout(()=>{wheelTimer=null;finish();},160);
    };
    const cancel=()=>{
      cancelFrame();if(wheelTimer!==null)host.clearTimeout(wheelTimer);
      wheelTimer=null;dragging=false;active=false;
    };
    for(const name of ['pointerdown','mousedown'])listen(node,name,down,true);
    for(const name of ['pointerup','pointercancel','mouseup'])listen(host,name,up,true);
    listen(node,'wheel',wheel,{capture:true,passive:true});
    listen(host,'blur',()=>{const wasActive=active;cancel();if(wasActive)settled();});
    return {get active(){return active;},cancel,dispose(){cancel();listeners.forEach(remove=>remove());}};
  }
  // Only the latest requested view may publish a completed plot, and Plotly
  // never receives overlapping newPlot calls for the same DOM node.
  function latestRenderer(draw) {
    let version=0, serial=Promise.resolve();
    return {
      request(value){
        const requested=++version;
        serial=serial.catch(()=>{}).then(async()=>{
          if(requested!==version)return false;
          await draw(value,()=>requested===version);
          return requested===version;
        });
        return serial;
      },
      cancel(){version++;}
    };
  }
  const stableScenes=new WeakSet();
  function commitCamera(node) {
    const scene=node._fullLayout?.scene?._scene;
    if(typeof scene?.getCamera!=='function')return;
    // Plotly normally commits only at mouse-up (and not outside the canvas).
    // Preserve the live pose before a data update can restore an older layout.
    const camera=scene.getCamera();
    node.layout.scene.camera=camera;scene.fullSceneLayout.camera=camera;
  }
  function stabilizeCamera(node) {
    const scene=node._fullLayout?.scene?._scene;
    if(!scene||stableScenes.has(scene)||typeof scene.updateFx!=='function'||!scene.camera?.view?.setMode)return;
    const original=scene.updateFx;
    // The bundled Plotly camera.mode='turntable' setter queues a 500 ms
    // transition, even when already in that mode. Every restyle calls it.
    // Use the underlying immediate mode switch, preserving Plotly's other
    // interaction modes and its upright turntable convention.
    scene.updateFx=function(mode,hover){
      if(mode!=='turntable')return original.call(this,mode,hover);
      this.camera.up=[0,0,1];this.camera.view.setMode('turntable');this.camera.keyBindingMode='rotate';
      this.fullSceneLayout.hovermode=hover;
      const current=this.fullSceneLayout.camera;
      if(current){
        current.up={x:0,y:0,z:1};
        const layout=this.graphDiv.layout[this.id];
        if(layout?.camera)layout.camera.up={x:0,y:0,z:1};
      }
    };
    stableScenes.add(scene);
  }
  const api={bindGestures,latestRenderer,stabilizeCamera,commitCamera};
  if(typeof module!=='undefined')module.exports=api;else window.missilePlot=api;
})();
