'use strict';
// Offline host publisher. Native Git/SquashFS tools do all bulk input work.
// Invoke under flock /opt/wt-em/shared/shared-inputs.lock; no per-version server.
const fs=require('node:fs'),path=require('node:path'),crypto=require('node:crypto');
const {spawn,execFileSync}=require('node:child_process');
const SHARED=process.env.WT_EM_SHARED||'/opt/wt-em/shared',BIN=process.env.WT_EM_INPUT_BIN||path.join(__dirname,'bin');
const datasets=path.join(SHARED,'datasets'),images=path.join(SHARED,'dataset-images'),catalogs=path.join(SHARED,'input-catalogs');
const mountsFile=path.join(SHARED,'dataset-mounts.json'),manifestFile=path.join(catalogs,'manifest.json');
const read=p=>JSON.parse(fs.readFileSync(p)),run=(name,args)=>execFileSync(name,args,{encoding:'utf8',maxBuffer:1000000}).trim();
const version=v=>/^\d+\.\d+\.\d+\.\d+$/.test(v),atomic=(p,value)=>{fs.writeFileSync(p+'.tmp',JSON.stringify(value),{mode:0o644});fs.renameSync(p+'.tmp',p);};
function mounted(p){try{run('mountpoint',['-q',p]);return true;}catch{return false;}}
function mountAll(){
 fs.mkdirSync(path.join(datasets,'.mounts'),{recursive:true});
 if(!mounted(datasets))run('mount',['--bind',datasets,datasets]);run('mount',['--make-shared',datasets]);
 for(const [name,record] of Object.entries(read(mountsFile))){
  if(!/^[a-z0-9.-]+$/.test(name)||!/^[a-z0-9.-]+\.sqfs$/.test(record.image))throw Error('Unsafe input mount');
  const target=path.join(datasets,'.mounts',name);fs.mkdirSync(target,{recursive:true});
  if(fs.lstatSync(target).isSymbolicLink())throw Error('Input mount cannot be a symbolic link');
  if(!mounted(target)){
   const image=path.join(images,record.image);if(record.sha256&&run('sha256sum',[image]).split(' ')[0]!==record.sha256)throw Error('Immutable input image checksum mismatch');
   run('mount',['-o','loop,ro',image,target]);
  }
 }
}
async function pack(store,spec,target){
 const producer=spawn(path.join(BIN,'dataset_pack'),[store,spec],{stdio:['ignore','pipe','pipe']});
 const consumer=spawn('mksquashfs',['-',target,'-tar','-comp','zstd','-Xcompression-level','3','-processors','1','-no-progress','-no-duplicates','-noappend'],{stdio:['pipe','pipe','pipe']});
 producer.stdout.pipe(consumer.stdin);consumer.stdin.on('error',()=>{});
 let logs='',failure;const stop=reason=>{failure=reason;producer.kill('SIGKILL');consumer.kill('SIGKILL');};
 for(const stream of [producer.stderr,consumer.stdout,consumer.stderr])stream.on('data',b=>{if(logs.length+b.length>1000000)stop('Publication log exceeded 1 MB');else logs+=b;});
 const timer=setInterval(()=>{if(fs.existsSync(target)&&fs.statSync(target).size>500000000)stop('Input image exceeded 500 MB');},250);
 try{const codes=await Promise.all([producer,consumer].map(p=>new Promise((resolve,reject)=>{p.once('error',reject);p.once('exit',resolve);})));if(failure||codes.some(c=>c!==0))throw Error(failure||logs.slice(-2000));}
 finally{clearInterval(timer);}
}
async function publish(v,prepared=null){
 if(!version(v))throw Error('Invalid input version');
 const manifest=read(manifestFile);if(!prepared&&manifest[v]&&fs.existsSync(path.join(datasets,manifest[v].data_path||v,'references/data-version.json')))return false;
 const store=path.join(SHARED,'version-store.git'),tree=prepared?.tree||run('git',['--git-dir',store,'rev-parse',v+'^{tree}']),spec=v+'='+tree;
 const name=v+'-'+tree.slice(0,12),image=path.join(images,name+'.sqfs'),target=path.join(datasets,'.mounts',name);
 let mounts=read(mountsFile);
 if(!mounts[name]){
  const pending=image+'.partial';if(fs.existsSync(pending))fs.unlinkSync(pending);
  try{await pack(store,spec,pending);fs.chmodSync(pending,0o644);fs.renameSync(pending,image);}
  catch(error){if(fs.existsSync(pending))fs.unlinkSync(pending);throw error;}
  fs.mkdirSync(target,{recursive:true});run('mount',['-o','loop,ro',image,target]);
  try{const checked=JSON.parse(run(path.join(BIN,'dataset_check'),[store,target,spec]));if(!checked.passed)throw Error('Input integrity failed');
   mounts[name]={image:path.basename(image),tree,versions:[v],bytes:fs.statSync(image).size,sha256:run('sha256sum',[image]).split(' ')[0],verification:checked};atomic(mountsFile,mounts);
  }catch(error){run('umount',[target]);throw error;}
 }
 const link=path.join(datasets,v),relative=path.join('.mounts',name,v);
 if(!fs.existsSync(link))fs.symlinkSync(relative,link);
 // Keep earlier snapshots available. A new generation uses its explicit path;
 // it never modifies the version symlink or files used by an existing job.
 const stage=path.join(catalogs,'.prepare-'+v);fs.mkdirSync(stage,{recursive:true});fs.chmodSync(stage,0o777);
 try{
  const runtime=process.env.WT_EM_PUBLICATION_IMAGE||run('docker',['inspect','--format','{{.Image}}','wt-em-app']);
  const output=run('docker',['run','--rm','--network','none','--memory','2g','--cpus','1',
   '-e','WT_EM_INPUT_ROOT=/datasets/.mounts/'+name+'/'+v,'-e','WT_EM_BUILD_SERVER=1','-e','WT_EM_OUTPUT_DIR=/tmp',
   '-v',datasets+':/datasets:ro','-v',stage+':/output',
   '-v',path.join(__dirname,'prepare_shared_catalog.py')+':/srv/wt-em/scripts/prepare_shared_catalog.py:ro',
   runtime,'python','scripts/prepare_shared_catalog.py','/output']);
  const record=JSON.parse(output.split('\n').at(-1));if(record.version!==v||record.key.build!==v)throw Error('Published catalog has wrong input identity');
  for(const [suffix,key] of [['.gz','snapshot_sha256'],['.json.gz','meta_sha256']]){
   const source=path.join(stage,v+suffix);if(crypto.createHash('sha256').update(fs.readFileSync(source)).digest('hex')!==record[key])throw Error('Published catalog checksum mismatch');
   const filename=v+'.'+record[key]+suffix;fs.renameSync(source,path.join(catalogs,filename));record[suffix==='.gz'?'snapshot_file':'meta_file']=filename;
  }
  delete record.version;atomic(manifestFile,{...read(manifestFile),[v]:{...record,tree,data_path:relative,current_fingerprint:prepared?.fingerprint}});
  console.log(JSON.stringify({version:v,result:'published',image_bytes:fs.statSync(image).size}));
 }finally{for(const suffix of ['.gz','.json.gz']){const p=path.join(stage,v+suffix);if(fs.existsSync(p))fs.unlinkSync(p);}fs.rmdirSync(stage);}
 return true;
}
async function publishCurrent(){
 // Called by the compiler hook while its update lock is held. Preserve the
 // newer geometry/model inputs of the current build without changing its tag.
 const references=path.join(SHARED,'references'),identity=()=>fs.readFileSync(path.join(references,'input-build.json'),'utf8')+fs.readFileSync(path.join(references,'data-version.json'),'utf8');
 const before=identity(),fingerprint=crypto.createHash('sha256').update(before).digest('hex'),v=read(path.join(references,'data-version.json')).version;
 if(!version(v))throw Error('Invalid current input version');if(read(manifestFile)[v]?.current_fingerprint===fingerprint)return false;
 const store=path.join(SHARED,'version-store.git'),index=path.join(SHARED,'.shared-input-index'),exclude=path.join(SHARED,'.shared-input-exclude');
 fs.writeFileSync(exclude,'/native/\n');if(fs.existsSync(index))fs.unlinkSync(index);
 const git=args=>execFileSync('git',['--git-dir',store,...args],{cwd:references,env:{...process.env,GIT_INDEX_FILE:index,GIT_WORK_TREE:references},encoding:'utf8',maxBuffer:1000000}).trim();
 try{
  git(['-c','core.excludesFile='+exclude,'add','--all','.']);const tree=git(['write-tree']);if(before!==identity())throw Error('Inputs changed during publication; retry after the compiler finishes');
  git(['update-ref','refs/shared-inputs/'+v+'/'+tree,tree]);return await publish(v,{tree,fingerprint});
 }finally{for(const file of [index,exclude])if(fs.existsSync(file))fs.unlinkSync(file);}
}
async function main(){mountAll();if(process.argv[2]==='--mount-only')return;
 if(process.argv[2]==='--primary'){console.log(JSON.stringify({current_prepared:await publishCurrent()}));return;}
 const requested=process.argv[2],all=requested?[requested]:run('git',['--git-dir',path.join(SHARED,'version-store.git'),'tag','--list']).split('\n').filter(version);
 let prepared=0;for(const v of all)if(await publish(v))prepared++;console.log(JSON.stringify({prepared,stored:all.length}));
}
if(require.main===module)main().catch(e=>{console.error(e.stack);process.exitCode=1;});
module.exports={mountAll,publish,publishCurrent};
