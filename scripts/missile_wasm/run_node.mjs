// Native live NDJSON protocol with a filesystem-backed MEMFS input manifest.
import fs from 'node:fs';
import path from 'node:path';
import readline from 'node:readline';
import {fileURLToPath} from 'node:url';
import createEngine from '../../app/live/missile-engine.mjs';
const root=path.resolve(process.argv[2]||fileURLToPath(new URL('../../',import.meta.url)));
const module=await createEngine({wasmBinary:fs.readFileSync(new URL('../../app/live/missile-engine.wasm',import.meta.url))});
const inputs=fs.existsSync(path.join(root,'references/missile-inputs/launch-profiles/index.json'))?'references/missile-inputs':'scripts/missile_model';
const index=JSON.parse(fs.readFileSync(path.join(root,inputs,'launch-profiles/index.json')));
for(const relative of [inputs+'/launch-profiles/index.json',inputs+'/radar-launch-properties.json',
  ...index.profiles.map(p=>inputs+'/launch-profiles/'+p.path),
  'scripts/missile_model/launch-spread-table.json','scripts/missile_model/motor-deviation-table.json']){
  module.FS.mkdirTree(path.posix.dirname('/eng/'+relative));module.FS.writeFile('/eng/'+relative,fs.readFileSync(path.join(root,relative)));
}
const call=(name,type,types,args)=>module.ccall(name,type,types,args);
const error=()=>call('last_error','string',[],[]);
if(call('engine_init','number',['string'],['/eng'])<0)throw Error(error());
for await(const line of readline.createInterface({input:process.stdin})){
  try{
    const request=JSON.parse(line);let reply;
    if(request.start){const handle=call('live_start','number',['string'],[JSON.stringify(request.start)]);if(handle<0)throw Error(error());reply={handle};}
    else if(request.step){const status=call('live_step','number',Array(5).fill('number'),request.step);if(status<0)throw Error(error());const offset=call('live_state_ptr','number',[],[])>>>3;reply={status,state:Array.from(module.HEAPF64.slice(offset,offset+32))};}
    else if(request.result){const value=call('live_result','string',['number'],[request.result]);if(!value)throw Error(error());console.log(value);continue;}
    else if(request.free){call('live_free',null,['number'],[request.free]);reply=JSON.parse(call('engine_info','string',[],[]));}
    else if(request.info)reply=JSON.parse(call('engine_info','string',[],[]));
    else throw Error('Unknown live command');
    console.log(JSON.stringify(reply));
  }catch(error){console.log(JSON.stringify({error:error.message}));}
}
