'use strict';
const fs=require('fs'),assert=require('assert/strict'),crypto=require('crypto');
const config=JSON.parse(fs.readFileSync(process.argv[2]));
const base=process.env.WT_EM_TEST_URL||'http://127.0.0.1:18872',old=process.env.WT_EM_REFERENCE_URL,active=new Set(['queued','running','exporting','stopping']);
const wait=ms=>new Promise(r=>setTimeout(r,ms)),hash=v=>crypto.createHash('sha256').update(JSON.stringify(v)).digest('hex');
async function api(path,body,host=base){const r=await fetch(host+path,{method:body?'POST':'GET',headers:{'Content-Type':'application/json'},body:body?JSON.stringify(body):undefined,signal:AbortSignal.timeout(90000)});const j=await r.json();assert(r.ok,JSON.stringify({path,status:r.status,j}));return j;}
async function finish(prefix,id,host=base){for(let i=0;i<900;i++){const j=await api(prefix+'/api/jobs/'+id,null,host);if(!active.has(j.status)){assert.equal(j.status,'complete',JSON.stringify(j));return j;}await wait(100);}throw Error('Job timeout');}
const chartParts=['boundary','contours','sustained_curve','numerical_boundaries','numerical_gaps','display_discontinuities','points','sustained','mass','engine'];
function physical(chart){return chart.aircraft.map(a=>Object.fromEntries(chartParts.map(k=>[k,a[k]])));}
async function calculate(version,host=base){const prefix=host===old?'':'/v/'+version;const t=performance.now();const {id}=await api(prefix+'/api/jobs',config,host);await finish(prefix,id,host);const chart=await api(prefix+'/api/jobs/'+id+'/chart.json',null,host);assert(chart.aircraft[0].boundary.length>2);assert(chart.aircraft[0].chart_solver_work.native.status.startsWith('enabled'));return {version,id,seconds:(performance.now()-t)/1000,chart};}
(async()=>{
 const report={passed:false,config_sha256:hash(config),metadata:0,calculations:[],checks:[]};
 const list=await api('/api/versions'),before=await api('/api/health');
 for(const b of list.builds.filter(b=>b.available)){const m=await api('/v/'+b.id+'/api/meta');assert.equal(m.game_data.version,b.data_build);assert(Object.keys(m.aircraft).length>100);report.metadata++;}
 assert.equal((await api('/api/health')).workers,before.workers);report.checks.push('All available aliases return their exact input catalog without creating workers');
 console.log(JSON.stringify({metadata:report.metadata}));
 const first=await calculate('2.56.0.47'),reference=old?await calculate('2.56.0.47',old):null;
 if(reference){assert.deepEqual(physical(first.chart),physical(reference.chart));report.checks.push('Compiled charts exactly equal the reference on identical inputs');}
 const prefix='/v/'+first.version+'/api/jobs/'+first.id;
 const point=await api(prefix+'/point/'+first.chart.aircraft[0].id+'/0');
 report.calculations.push({version:first.version,id:first.id,seconds:first.seconds,physical_sha256:hash(physical(first.chart))});
 for(const version of ['2.56.0.46','2.59.0.94','2.61.0.9']){const r=await calculate(version);report.calculations.push({version,id:r.id,seconds:r.seconds,physical_sha256:hash(physical(r.chart))});console.log(JSON.stringify({version,seconds:r.seconds}));}
 assert.equal((await api('/api/health')).workers,3);
 assert.deepEqual(physical(await api(prefix+'/chart.json')),physical(first.chart));
 assert.deepEqual(await api(prefix+'/point/'+first.chart.aircraft[0].id+'/0'),point);report.checks.push('Completed charts and point details survive eviction and restoration across four versions');
 const engine=await api('/v/2.56.0.47/api/engine/mig-19s?sweep=speed&altitude_m=0&speed_kmh=300');assert(Object.keys(engine).length>0);report.checks.push('Engine endpoint');
 const missile=await api('/v/2.56.0.47/api/missiles/meta');assert(missile.missiles.length>10);report.checks.push('Version-specific missile catalog');
 const cancelled=await api('/v/2.56.0.47/api/jobs',config);
 assert((await api('/v/2.56.0.47/api/jobs/'+cancelled.id+'/cancel',{})).cancelled);
 for(let i=0;i<300;i++){const j=await api('/v/2.56.0.47/api/jobs/'+cancelled.id);if(!active.has(j.status)){assert.equal(j.status,'stopped');break;}assert(i<299);await wait(100);}
 const pending=await Promise.all(['2.56.0.46','2.59.0.94','2.61.0.9'].map(async v=>[v,await api('/v/'+v+'/api/jobs',config)]));
 await Promise.all(pending.map(([v,j])=>finish('/v/'+v,j.id)));
 assert((await api('/api/health')).workers<=3);report.checks.push('Concurrent jobs, cancellation, and bounded worker count');
 report.passed=true;console.log('REPORT '+JSON.stringify(report));
})().catch(e=>{console.error(e.stack);process.exitCode=1;});
