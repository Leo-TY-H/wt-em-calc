// Standalone calculation entry point and native envelope scheduler.
#include "runtime.h"
#include "engine_stdlib.h"
#include "engine_kernels.h"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <future>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#else
#include <cerrno>
#include <signal.h>
#include <unistd.h>
#endif

void register_generated_modules();
namespace nt { void release_thread_runtime(); void native_profile_report(); }
using namespace nt;
using Clock=std::chrono::steady_clock;
static double seconds(Clock::time_point t){return std::chrono::duration<double>(Clock::now()-t).count();}
static V member(V v,const char* name){return item(v,name);}
static V invoke(const char* module,const char* function,Args args={},Kw kwargs={}){return call(attr(import_module(module),function),args,kwargs);}
static void send(const char* kind,V value){std::cout<<json_dump(make_dict({{"type",kind},{kind,value}}),false,false)<<std::endl;}

class Pool {
 std::vector<std::thread> threads;std::queue<std::function<void()>> queue;std::mutex mutex;std::condition_variable cv;bool stopping=false;
public:
 explicit Pool(int count){for(int i=0;i<count;++i)threads.emplace_back([this]{for(;;){std::function<void()> task;{std::unique_lock<std::mutex> lock(mutex);cv.wait(lock,[&]{return stopping||!queue.empty();});if(stopping&&queue.empty())break;task=std::move(queue.front());queue.pop();}task();}clear_kernel_caches();release_thread_runtime();});}
 std::future<V> submit(std::function<V()> fn){auto task=std::make_shared<std::packaged_task<V()>>(std::move(fn));auto future=task->get_future();{std::lock_guard<std::mutex> lock(mutex);queue.emplace([task]{(*task)();});}cv.notify_one();return future;}
 ~Pool(){{std::lock_guard<std::mutex> lock(mutex);stopping=true;}cv.notify_all();for(auto& t:threads)t.join();}
};

static V evaluation(V config,V layer,V azimuth,V distance){
 import_module("missile_envelope");
 InvocationGuard scope;
 V actors=invoke("missile_envelope","states",{config,layer,azimuth,distance});
 V reach=member(config,"criterion").as_string()=="kinematic"?member(config,"reach_m"):V();
 try{return invoke("missile_envelope","evaluate",{member(config,"missile"),item(actors,0),item(actors,1),member(config,"early_exit"),reach});}
 catch(const PyError& e){return make_dict({{"hit",false},{"outcome","error"},{"error",e.kind+": "+e.what()},{"time_s",V()},{"closest_m",V()}});}
}

static V envelope(V request){
 auto start=Clock::now();V config=invoke("missile_envelope","validate",{request});
 V limits=invoke("missile_envelope","missile_limits",{member(config,"missile")});
 if(member(config,"criterion").as_string()=="kinematic"&&member(config,"reach_m").is_none()){
  V reach=member(limits,"fuze_radius_m");setitem(config,"reach_m",truth(reach)?reach:V(5.));
 }
 Args rays=iter(invoke("missile_envelope","plan",{config,limits}));V angles=invoke("missile_envelope","azimuths",{config});
 V mirrored=invoke("missile_envelope","mirrored",{config}),ray_map=make_dict();
 for(V ray:rays)dict_set(ray_map,attr(ray,"key"),ray);
 V grid=call(attr(import_module("missile_envelope"),"Grid"),{ray_map,V(int(iter(angles).size())),V(!truth(mirrored)),angles});
 int workers=int(invoke("missile_envelope","default_workers").as_int());
 std::atomic<int64_t> simulations{0},errors{0},early{0};
 Pool samples(workers),searches(workers);
 send("plan",make_dict({{"config",config},{"limits",limits},{"rays_total",int(rays.size())}}));
 struct Pending {size_t index;std::future<V> result;};
 std::vector<Pending> pending;std::vector<bool> started(rays.size(),false);size_t complete=0;
 std::vector<int64_t> depth;V memo=make_dict();for(V ray:rays)depth.push_back(call(attr(grid,"depth"),{attr(ray,"key"),memo}).as_int());
 std::vector<size_t> order;for(size_t i=0;i<rays.size();++i)order.push_back(i);
 std::stable_sort(order.begin(),order.end(),[&](size_t a,size_t b){return depth[a]<depth[b];});
 while(complete<rays.size()){
  for(size_t index:order){
   if(pending.size()>=size_t(workers))break;
   if(started[index]||!truth(call(attr(grid,"ready"),{attr(rays[index],"key")})))continue;
   started[index]=true;V seed=call(attr(grid,"seed"),{rays[index]}),ray=deepcopy(rays[index]);
   setattr(ray,"seed_source",truth(seed)?deepcopy(seed):V());
   pending.push_back({index,searches.submit([&,ray,seed,config]{
    auto began=Clock::now();InvocationGuard scope;
    envelope_yield=[&,ray,config](V ranges){
     Args distances=iter(ranges);simulations+=int64_t(distances.size());
     setattr(ray,"simulations",attr(ray,"simulations").as_int()+int64_t(distances.size()));
     std::vector<std::future<V>> futures;
     for(V r:distances)futures.push_back(samples.submit([=]{return evaluation(config,attr(ray,"layer"),attr(ray,"azimuth"),r);}));
     Args outcomes;for(auto& future:futures){V outcome=future.get();std::string kind=member(outcome,"outcome").as_string();errors+=kind=="error";early+=kind=="unreachable";outcomes.push_back(outcome);}return make_list(outcomes);
    };
    try{call(attr(ray,"search"),{seed});}catch(...){envelope_yield={};throw;}
    envelope_yield={};V result=attr(ray,"result");dict_set(result,"wall_s",seconds(began));
    int count=0;for(V sample:iter(member(result,"samples")))count+=member(sample,"outcome").as_string()=="error";
    dict_set(result,"errors",count);return result;
   })});
  }
  bool changed=false;
  for(size_t i=0;i<pending.size();){
   if(pending[i].result.wait_for(std::chrono::milliseconds(0))!=std::future_status::ready){++i;continue;}
   V result=pending[i].result.get();setattr(rays[pending[i].index],"result",result);pending.erase(pending.begin()+i);++complete;changed=true;
   send("ray",result);send("progress",make_dict({{"rays_done",int(complete)},{"rays_total",int(rays.size())},{"simulations",simulations.load()},{"elapsed_s",seconds(start)}}));
  }
  if(pending.empty()&&complete<rays.size()&&!changed)throw PyError("RuntimeError","Envelope search plan has no ready direction");
  if(!changed)std::this_thread::sleep_for(std::chrono::milliseconds(1));
 }
 return make_dict({{"format","missile-envelope-v1"},{"config",config},{"limits",limits},{"mirrored",mirrored},
  {"azimuths_deg",angles},{"simulations",simulations.load()},{"sample_errors",errors.load()},{"early_misses",early.load()},
  {"elapsed_s",seconds(start)},{"workers",workers}});
}

static void watch_parent(V request){
 if(request.kind()!=Kind::Dict)return;
 int64_t previous=0;
 for(const char* key:{"_parent_pid","_supervisor_pid"}){
 V pid=dict_get(request,key);if(pid.is_none())continue;
 int64_t id=pid.as_int();if(id==previous)continue;if(id<=0)throw PyError("InterruptedError","Parent process is unavailable");previous=id;
#ifdef _WIN32
 HANDLE process=OpenProcess(SYNCHRONIZE,FALSE,DWORD(id));if(!process)throw PyError("InterruptedError","Parent process is unavailable");
 std::thread([process]{WaitForSingleObject(process,INFINITE);CloseHandle(process);ExitProcess(2);}).detach();
#else
 int parent=int(id);std::thread([parent]{for(;;){if(kill(parent,0)!=0&&errno==ESRCH)_exit(2);std::this_thread::sleep_for(std::chrono::milliseconds(100));}}).detach();
#endif
 }
}

int main(int argc,char**argv){
 struct ProfileReport {~ProfileReport(){native_profile_report();}} profile_report;
 std::string root=std::filesystem::absolute(argv[0]).parent_path().parent_path().string(),mode="simulator";bool batch=false;
 for(int i=1;i<argc;++i){std::string a=argv[i];if(a=="--root"&&i+1<argc)root=argv[++i];else if(a=="--mode"&&i+1<argc)mode=argv[++i];else if(a=="--batch")batch=true;else if(a=="--version"){std::cout<<"wt-native-engine-v1; runtime=C++17; python=false\n";return 0;}else{std::cerr<<"Unknown engine argument\n";return 2;}}
 try{
  register_native_stdlib(root);register_generated_modules();
  auto run=[&](const std::string& data){
   V request=json_parse(data);watch_parent(request);
   // Load modules outside the invocation cleanup boundary.
   import_module(mode=="envelope"||mode=="envelope-setup"||mode=="evaluate"?"missile_envelope":mode=="call"?member(request,"module").as_string():"missile_worker");
   InvocationGuard scope;V result;
   if(mode=="envelope")result=envelope(request);
   else if(mode=="envelope-setup"){V c=invoke("missile_envelope","validate",{request}),a=invoke("missile_envelope","azimuths",{c});result=make_dict({{"config",c},{"azimuths_deg",a},{"mirrored",invoke("missile_envelope","mirrored",{c})},{"rays_total",int(iter(member(c,"layers_m")).size()*iter(a).size())}});}
   else if(mode=="evaluate")result=invoke("missile_envelope","evaluate",{member(request,"missile"),member(request,"launcher"),member(request,"target"),dict_get(request,"early_exit",true),dict_get(request,"reach_m")});
   else if(mode=="call"){Kw kwargs;V k=dict_get(request,"kwargs",make_dict());for(auto& p:k.node()->entries)kwargs.emplace_back(p.first.as_string(),p.second);result=call(attr(import_module(member(request,"module").as_string()),member(request,"function").as_string()),iter(dict_get(request,"args",make_list())),kwargs);}
   else if(mode=="simulator")result=invoke("missile_worker","simulate",{request,builtin("progress",[&](const Args&a,const Kw&){if(!batch)send("progress",a.at(0));return V();})});
   else throw PyError("ValueError","Unknown calculation mode");
   send("result",result);
  };
  if(batch){std::string line;while(std::getline(std::cin,line)){if(line.empty())continue;try{run(line);}catch(const PyError&e){send("error",e.kind+": "+e.what());}}}
  else{std::string data(std::istreambuf_iterator<char>(std::cin),{});run(data);}
  return 0;
 }catch(const PyError&e){send("error",e.kind+": "+e.what());return 1;}
 catch(const std::exception&e){send("error",std::string("RuntimeError: ")+e.what());return 1;}
}
