'use strict';
// Run with node scripts/test_game_versions.cjs. No browser or network required.
const assert=require('node:assert/strict');
const {readFileSync}=require('node:fs');
const vm=require('node:vm');
const {test}=require('node:test');
const source=readFileSync(require('node:path').join(__dirname,'../app/versions.js'),'utf8');
const primary='2.60.0.35',latest='2.61.0.10';
const row=(id,available=true,data_build=id)=>({id,available,data_build});
const listing=()=>({default:primary,rules:'2.60',builds:[row('2.61.0.9'),row('2.62.0.1',false),row(primary),row(latest)]});
const response=(value,status=200)=>({ok:status>=200&&status<300,status,json:async()=>value});
function harness(value=listing(),handle=()=>response({aircraft:{},version:'meta'})){
  const calls=[];
  const context=vm.createContext({window:{EM_CONFIG:{apiBase:'https://api.test'}},performance:{now:()=>Date.now()},
    setTimeout:fn=>queueMicrotask(fn),fetch:async(url,options)=>{
      calls.push({url,options});
      return url.endsWith('/api/versions')?response(value):handle(url,options);
    }});
  vm.runInContext(source,context);
  return {g:context.window.GameVersions,calls};
}
test('newest built numeric version wins regardless of listing order; concurrent loads share one request',async()=>{
  const {g,calls}=harness();await Promise.all([g.load(),g.load(),g.load()]);
  assert.equal(calls.length,1);assert.equal(g.default,latest);
  assert.equal(g.url('/api/jobs'),'https://api.test/v/'+latest+'/api/jobs');
  assert.equal(g.url('/api/jobs',primary),'https://api.test/api/jobs');
  assert.equal(g.url('/api/jobs','2.61.0.9'),'https://api.test/v/2.61.0.9/api/jobs');
});
test('default and historical primary catalogs use their own servers',async()=>{
  const {g,calls}=harness();await g.load();
  assert.ok(await g.ensure());assert.ok(await g.ensure(primary));
  assert.ok(g.meta(primary));assert.equal(g.meta(),null);
  assert.equal(calls[1].url,'https://api.test/v/'+latest+'/api/meta');
  assert.equal(calls[2].url,'https://api.test/api/meta');
  await g.request('/api/jobs',{},primary);
  assert.equal(calls.at(-1).url,'https://api.test/api/jobs');
});
test('an idle default server starts again before retrying the calculation',async()=>{
  let running=true,failed=false;
  const {g,calls}=harness(listing(),url=>{
    if(url.endsWith('/start')){running=true;return response({});}
    if(url.endsWith('/api/meta'))return response({},running?200:503);
    if(!failed){failed=true;running=false;return response({},503);}
    return response({calculated:true});
  });
  await g.load();const result=await g.request('/api/jobs',{method:'POST'});
  assert.equal((await result.json()).calculated,true);
  assert.equal(calls.filter(c=>c.url.endsWith('/start')).length,1);
  assert.equal(calls.find(c=>c.url.endsWith('/start')).url,'https://api.test/api/versions/'+latest+'/start');
});
test('newest aliases share the canonical data server',async()=>{
  const data=listing();data.builds.find(b=>b.id===latest).data_build='2.61.0.9';
  const {g}=harness(data);await g.load();assert.equal(g.default,latest);
  assert.equal(g.url('/api/meta'),'https://api.test/v/2.61.0.9/api/meta');
  assert.equal(g.isDefault('2.61.0.9'),true);
});
test('newest alias of primary stays on the unversioned API',async()=>{
  const data=listing();data.builds.find(b=>b.id===latest).data_build=primary;
  const {g}=harness(data);await g.load();assert.equal(g.default,latest);
  assert.equal(g.url('/api/meta'),'https://api.test/api/meta');assert.equal(await g.ensure(),null);
});
test('explicit older selections stay separate from the new default in comparisons',async()=>{
  const {g}=harness();await g.load();
  const result=g.split({entries:[{id:'a',aircraft_id:'f16'},{id:'b',aircraft_id:'f16',version:primary},{id:'c',aircraft_id:'f16',version:latest}]});
  assert.equal(result.length,2);assert.equal(result[0].version,undefined);
  assert.equal(result[0].config.entries.length,2);assert.equal(result[1].version,primary);
});
test('local installations without a version list keep their ordinary API',async()=>{
  const {g}=harness({});await g.load();assert.equal(g.enabled(),false);
  assert.equal(g.url('/api/meta'),'https://api.test/api/meta');assert.equal(await g.ensure(),null);
});
