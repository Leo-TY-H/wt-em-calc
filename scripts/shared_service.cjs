'use strict';
// One HTTP service, immutable version inputs, and a bounded pool of pipe workers.
// No version-specific listening ports, containers, or request-time data extraction.
const http=require('node:http'),fs=require('node:fs'),path=require('node:path');
const zlib=require('node:zlib'),crypto=require('node:crypto'),net=require('node:net'),{spawn}=require('node:child_process');
const ROOT=path.resolve(__dirname,'..'),DATA=process.env.WT_EM_DATASETS;
const CATALOGS=process.env.WT_EM_CATALOGS;
const WORKER_SOCKET=process.env.WT_EM_WORKER_SOCKET||'/tmp/wt-em-workers.sock';
if(require.main===module&&(!DATA||!CATALOGS)){
  const local=spawn(process.env.WT_EM_PYTHON||'python',[path.join(__dirname,'em_server.py'),...process.argv.slice(2)],{stdio:'inherit',env:process.env});
  local.on('exit',code=>{process.exitCode=code||0;});local.on('error',error=>{console.error(error);process.exitCode=1;});
  for(const signal of ['SIGTERM','SIGINT'])process.on(signal,()=>local.kill(signal));
  return;
}
const LIMIT=3,SNAPSHOT_LIMIT=512000000,REQUEST_LIMIT=1000000;
const ACTIVE=new Set(['queued','running','stopping','exporting']);
const release={...JSON.parse(fs.readFileSync(path.join(ROOT,'app/release.json'))),...JSON.parse(fs.readFileSync(path.join(ROOT,'build-info.json')))};
release.tag='v'+release.version;release.source_url=release.repository+'/tree/'+release.tag;release.release_url=release.repository+'/releases/tag/'+release.tag;
const allowed=new Set((process.env.WT_EM_ALLOWED_ORIGINS||'').split(',').filter(Boolean));
let listingStamp='',listing=null,catalogManifest=null,lastRequest=performance.now(),serial=0,nativeVerified=false,factory=null,stopping=false;
const catalogs=new Map(),contexts=new Map(),waiters=[];
function dataRoot(version,catalog=catalogManifest[version]){return path.join(DATA,catalog?.data_path||version);}
function catalogFile(version,kind,catalog=catalogManifest[version]){return path.join(CATALOGS,catalog?.[kind+'_file']||(version+(kind==='meta'?'.json.gz':'.gz')));}
function versions(){
  const filename=path.join(ROOT,'.data-sync/versions.json'),stamp=fs.statSync(filename).mtimeMs+':'+fs.statSync(path.join(CATALOGS,'manifest.json')).mtimeMs;
  if(stamp!==listingStamp){
    catalogs.clear();
    catalogManifest=JSON.parse(fs.readFileSync(path.join(CATALOGS,'manifest.json')));
    listing=JSON.parse(fs.readFileSync(filename));const ready=new Map();
    listing.builds=listing.builds.map(b=>{if(!ready.has(b.data_build))ready.set(b.data_build,
      catalogManifest[b.data_build]?.key?.build===b.data_build&&fs.existsSync(path.join(dataRoot(b.data_build),'references/data-version.json'))&&fs.existsSync(catalogFile(b.data_build,'meta'))&&fs.existsSync(catalogFile(b.data_build,'snapshot')));
      return {...b,available:!!b.available&&ready.get(b.data_build)};});listingStamp=stamp;
    // Keep unversioned requests from already-open tabs bound to the same inputs.
    // The browser independently chooses the newest prepared version as its default.
    if(process.env.WT_EM_PRIMARY)listing.default=process.env.WT_EM_PRIMARY;
  }
  return listing;
}
function resolveVersion(id){
  const list=versions(),entry=list.builds.find(b=>b.id===(id||list.default)&&b.available);
  if(!entry)throw Object.assign(Error('Unknown or unavailable FM version'),{status:404});
  if(!/^\d+\.\d+\.\d+\.\d+$/.test(entry.data_build))throw Error('Invalid dataset identifier');
  if(!fs.existsSync(path.join(dataRoot(entry.data_build),'references/data-version.json')))throw Object.assign(Error('FM inputs are not ready'),{status:503});
  return entry.data_build;
}
function meta(version){
  if(catalogs.has(version)){const value=catalogs.get(version);catalogs.delete(version);catalogs.set(version,value);return value;}
  versions();
  const bytes=fs.readFileSync(catalogFile(version,'meta'));
  if(crypto.createHash('sha256').update(bytes).digest('hex')!==catalogManifest[version]?.meta_sha256)throw Error('Catalog metadata checksum mismatch');
  const value=JSON.parse(zlib.gunzipSync(bytes));
  if(value.game_data.version!==version)throw Error('Catalog input identity mismatch');
  value.release=release;value.result_policy={storage:'memory',fresh_calculations:true,automatic_saving:false};
  catalogs.set(version,value);while(catalogs.size>8)catalogs.delete(catalogs.keys().next().value);
  return value;
}
function wake(){for(const resolve of waiters.splice(0))resolve();}
function context(version){
  versions();const catalog=catalogManifest[version],key=version+':'+catalog.snapshot_sha256+':'+(catalog.tree||'');
  let value=contexts.get(key);if(!value){value={version,catalog,worker:null,snapshot:null,leases:0,busy:false,last:performance.now(),jobs:[]};contexts.set(key,value);}return value;
}
class Worker{
  constructor(owner){
    this.owner=owner;this.pending=new Map();this.buffer='';this.closed=false;
    const catalog=owner.catalog;
    if(catalog?.key?.build!==owner.version)throw Error('Catalog manifest input identity mismatch');
    const checksum=crypto.createHash('sha256').update(fs.readFileSync(catalogFile(owner.version,'snapshot',catalog))).digest('hex');
    if(checksum!==catalog.snapshot_sha256)throw Error('Catalog snapshot checksum mismatch');
    const env={WT_EM_INPUT_ROOT:dataRoot(owner.version,catalog),WT_EM_BUILD_SERVER:'1',
      WT_EM_START_CACHE:catalogFile(owner.version,'snapshot',catalog),WT_EM_CATALOG_COMMIT:catalog.key.commit,WT_EM_MAX_PENDING_JOBS:'2'};
    this.stream=net.createConnection(WORKER_SOCKET);this.stream.setEncoding('utf8');
    this.stream.write(JSON.stringify({environment:env})+'\n');
    this.stream.on('data',chunk=>{
      this.buffer+=chunk;if(this.buffer.length>768000000){this.fail(Error('Worker response too large'));this.stream.destroy();if(this.pid)process.kill(this.pid,'SIGKILL');return;}
      let at;while((at=this.buffer.indexOf('\n'))>=0){const line=this.buffer.slice(0,at);this.buffer=this.buffer.slice(at+1);let value;
        try{value=JSON.parse(line);}catch{this.fail(Error('Invalid worker response'));continue;}
        if(value.ready){this.pid=value.pid;continue;}
        const pending=this.pending.get(value.id);this.pending.delete(value.id);
        if(value.jobs){owner.jobs=value.jobs;owner.busy=value.jobs.some(j=>ACTIVE.has(j.status));if(!owner.busy)wake();}
        if(pending)value.error?pending.reject(Error(value.error)):pending.resolve(value);
      }
    });
    this.stream.on('error',e=>this.fail(e));this.stream.on('close',()=>this.fail(Error('Calculation worker stopped')));
  }
  fail(error){this.closed=true;for(const p of this.pending.values())p.reject(error);this.pending.clear();if(this.owner.worker===this){this.owner.worker=null;this.owner.busy=false;wake();}}
  call(command){if(this.closed)return Promise.reject(Error('Calculation worker closed'));return new Promise((resolve,reject)=>{
    const id=++serial;this.pending.set(id,{resolve,reject});this.stream.write(JSON.stringify({...command,id})+'\n',error=>{if(error){this.pending.delete(id);reject(error);}});
  });}
  close(){return new Promise(resolve=>{if(this.closed)return resolve();const timer=setTimeout(()=>{if(this.pid)try{process.kill(this.pid,'SIGKILL');}catch{}this.stream.destroy();},15000);timer.unref();this.stream.once('close',()=>{clearTimeout(timer);resolve();});this.stream.end(JSON.stringify({id:++serial,op:'close'})+'\n');});}
}
let allocation=Promise.resolve();
function allocate(action){const next=allocation.then(action,action);allocation=next.catch(()=>{});return next;}
function trimSnapshots(){
  let total=[...contexts.values()].reduce((n,c)=>n+(c.snapshot?.length||0),0);
  for(const c of [...contexts.values()].filter(c=>!c.worker&&!c.leases).sort((a,b)=>a.last-b.last)){
    if(total<=SNAPSHOT_LIMIT)break;total-=c.snapshot?.length||0;c.snapshot=null;c.jobs=[];
  }
}
async function acquire(version,selected=null){
  for(;;){
    const result=await allocate(async()=>{
      const c=selected||context(version);c.last=performance.now();
      if(c.worker){c.leases++;return c;}
      const running=[...contexts.values()].filter(v=>v.worker);
      if(running.length>=LIMIT){
        const idle=running.filter(v=>!v.busy&&!v.leases).sort((a,b)=>a.last-b.last)[0];if(!idle)return null;
        const previous=idle.worker;idle.leases++;
        try{const saved=await previous.call({op:'snapshot'});idle.snapshot=saved.snapshot;await previous.close();idle.worker=null;}
        finally{idle.leases--;}
      }
      c.leases++;
      try{c.worker=new Worker(c);if(c.snapshot){await c.worker.call({op:'restore',snapshot:c.snapshot});c.snapshot=null;}}
      catch(error){c.leases--;if(c.worker)await c.worker.close();c.worker=null;throw error;}
      trimSnapshots();return c;
    });
    if(result)return result;
    await new Promise(resolve=>{waiters.push(resolve);setTimeout(()=>{const i=waiters.indexOf(resolve);if(i>=0)waiters.splice(i,1);resolve();},500).unref();});
  }
}
async function rpc(version,command){
  // An input refresh affects new jobs; existing job URLs retain their inputs.
  const id=/^\/api\/(?:jobs|missiles\/jobs|missiles\/envelopes)\/([a-f0-9]+)(?:\/|\?|$)/.exec(command.path||'')?.[1];
  const selected=id?[...contexts.values()].find(c=>c.version===version&&c.jobs.some(j=>j.id===id)):null;
  const c=await acquire(version,selected);try{return await c.worker.call(command);}finally{c.leases--;c.last=performance.now();wake();}
}
function send(res,status,body,headers={}){res.writeHead(status,{'Cache-Control':'no-store',...headers,'Content-Length':body.length});res.end(body);}
function json(res,status,value){send(res,status,Buffer.from(JSON.stringify(value)),{'Content-Type':'application/json; charset=utf-8'});}
async function readBody(req){let size=0;const parts=[];for await(const part of req){size+=part.length;if(size>REQUEST_LIMIT)throw Object.assign(Error('Request too large'),{status:413});parts.push(part);}return Buffer.concat(parts);}
async function handle(req,res){
  const origin=req.headers.origin;if(origin&&allowed.has(origin)){res.setHeader('Access-Control-Allow-Origin',origin);res.setHeader('Vary','Origin');}
  if(req.method==='OPTIONS'){if(origin&&!allowed.has(origin))return json(res,403,{error:'Origin not allowed'});res.setHeader('Access-Control-Allow-Methods','GET, POST, OPTIONS');res.setHeader('Access-Control-Allow-Headers','Content-Type');res.setHeader('Access-Control-Max-Age','86400');res.writeHead(204);return res.end();}
  if(origin&&!allowed.has(origin))return json(res,403,{error:'Origin not allowed'});
  const url=new URL(req.url,'http://localhost'),match=/^\/v\/(\d+\.\d+\.\d+\.\d+)(\/api\/.*)$/.exec(url.pathname);
  const endpoint=match?match[2]:url.pathname;
  if(endpoint!=='/api/health')lastRequest=performance.now();
  if(endpoint==='/api/health')return json(res,nativeVerified?200:503,{status:nativeVerified?'ok':'starting',release,march_native_verified:nativeVerified,busy:[...contexts.values()].some(c=>c.busy||c.leases),idle_s:(performance.now()-lastRequest)/1000,shared_version_service:true,workers:[...contexts.values()].filter(c=>c.worker).length,worker_limit:LIMIT,game_data:meta(resolveVersion(match?.[1])).game_data});
  if(endpoint==='/api/versions')return json(res,200,versions());
  if(endpoint==='/api/version-history')return send(res,200,fs.readFileSync(path.join(ROOT,'.data-sync/version-history.json')),{'Content-Type':'application/json'});
  const start=/^\/api\/versions\/(\d+\.\d+\.\d+\.\d+)\/start$/.exec(endpoint);
  if(start&&req.method==='POST'){const version=resolveVersion(start[1]);return json(res,202,{version,ready:true});}
  const version=resolveVersion(match?.[1]);
  if(endpoint==='/api/meta'&&req.method==='GET')return json(res,200,meta(version));
  if(!endpoint.startsWith('/api/')||!['GET','POST'].includes(req.method))return json(res,404,{error:'Not found'});
  const body=await readBody(req);
  const reply=await rpc(version,{method:req.method,path:endpoint+url.search,body:body.toString('base64'),headers:origin?{Origin:origin}:{}});
  const response=reply.response;send(res,response.status,Buffer.from(response.body,'base64'),response.headers);
}
const timer=setInterval(()=>{for(const c of contexts.values())if(c.worker&&c.busy&&!c.leases){c.leases++;c.worker.call({op:'stats'}).catch(e=>console.error(e.message)).finally(()=>{c.leases--;wake();});}},300);timer.unref();
const server=http.createServer((req,res)=>handle(req,res).catch(error=>{console.error(error.message);if(!res.headersSent)json(res,error.status||500,{error:error.message});else res.destroy();}));
async function startFactory(){
  factory=spawn(process.env.WT_EM_PYTHON||'python',[path.join(__dirname,'worker_factory.py')],{env:{...process.env,WT_EM_WORKER_SOCKET:WORKER_SOCKET},stdio:['ignore','pipe','inherit']});
  await new Promise((resolve,reject)=>{let buffer='';factory.stdout.setEncoding('utf8');factory.stdout.on('data',chunk=>{buffer+=chunk;if(buffer.includes('\n')){try{if(JSON.parse(buffer.trim()).ready)resolve();}catch(error){reject(error);}}});factory.once('error',reject);factory.once('exit',code=>reject(Error('Worker template stopped ('+code+')')));});
  factory.on('exit',()=>{if(!stopping){console.error('Worker template stopped');process.exit(1);}});
}
if(require.main===module){
  if(!DATA||!CATALOGS)throw Error('WT_EM_DATASETS and WT_EM_CATALOGS are required');
  startFactory().then(()=>server.listen(+(process.env.PORT||8080),process.env.WT_EM_HOST||'0.0.0.0',async()=>{
    try{const reply=await rpc(resolveVersion(),{method:'GET',path:'/api/health'});const health=JSON.parse(Buffer.from(reply.response.body,'base64'));
      if(!health.march_native_verified)throw Error('Compiled solver verification is missing');nativeVerified=true;console.log('Shared calculation service ready');
    }catch(error){console.error(error);process.exit(1);}
  })).catch(error=>{console.error(error);process.exit(1);});
}
process.on('SIGTERM',async()=>{stopping=true;server.close();await Promise.all([...contexts.values()].filter(c=>c.worker).map(c=>c.worker.close()));factory?.kill();});
module.exports={resolveVersion,meta,server,contexts,acquire,rpc};
