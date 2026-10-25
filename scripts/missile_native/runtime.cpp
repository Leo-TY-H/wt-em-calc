#include "runtime.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>
#include <unordered_set>
#include <cctype>
#include <charconv>
#include <cstdlib>
#include <cerrno>
#include <chrono>
#include <iostream>
#include <mutex>
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace nt {
namespace {
using ProfileClock=std::chrono::steady_clock;
struct ProfileStat { uint64_t calls=0, inclusive_ns=0, exclusive_ns=0; };
struct ProfileScope;
struct ProfileThread {
 bool enabled=[](){const char* value=std::getenv("WT_NATIVE_PROFILE");return value&&std::strcmp(value,"1")==0;}();
 ProfileScope* current=nullptr;
 std::unordered_map<std::string,ProfileStat> rows;
 uint64_t omitted_calls=0;
};
ProfileThread& profile_thread(){static thread_local ProfileThread state;return state;}
struct ProfileTotals {std::mutex mutex;std::unordered_map<std::string,ProfileStat> rows;uint64_t omitted_calls=0;};
ProfileTotals& profile_totals(){static ProfileTotals state;return state;}
struct ProfileScope {
 ProfileThread* state=nullptr;ProfileScope* parent=nullptr;ProfileStat* stat=nullptr;
 ProfileClock::time_point started;uint64_t children=0;
 explicit ProfileScope(const Node& function){
  auto& local=profile_thread();if(!local.enabled)return;state=&local;
  std::string name=function.builtin?"builtin.":"";
  if(function.closure){for(auto env=function.closure;env;env=env->parent){auto it=env->values.find("__name__");if(it!=env->values.end()){name=it->second.as_string()+".";break;}}}
  name+=function.text;name.resize(std::min<size_t>(name.size(),160));
  auto found=local.rows.find(name);
  if(found==local.rows.end()&&local.rows.size()<2048)found=local.rows.emplace(name,ProfileStat{}).first;
  if(found!=local.rows.end())stat=&found->second;else ++local.omitted_calls;
  parent=local.current;local.current=this;started=ProfileClock::now();
 }
 ~ProfileScope(){if(!state)return;uint64_t ns=uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(ProfileClock::now()-started).count());
  if(stat){++stat->calls;stat->inclusive_ns+=ns;stat->exclusive_ns+=ns-std::min(ns,children);}
  state->current=parent;if(parent)parent->children+=ns;
 }
};
void profile_merge_thread(){auto& local=profile_thread();if(!local.enabled)return;auto& totals=profile_totals();std::lock_guard<std::mutex> lock(totals.mutex);
 totals.omitted_calls+=local.omitted_calls;for(const auto& pair:local.rows){auto found=totals.rows.find(pair.first);if(found==totals.rows.end()&&totals.rows.size()<2048)found=totals.rows.emplace(pair.first,ProfileStat{}).first;
  if(found==totals.rows.end()){totals.omitted_calls+=pair.second.calls;continue;}auto& out=found->second;out.calls+=pair.second.calls;out.inclusive_ns+=pair.second.inclusive_ns;out.exclusive_ns+=pair.second.exclusive_ns;}
 local.rows.clear();local.omitted_calls=0;
}
}
void native_profile_report(){if(!profile_thread().enabled)return;profile_merge_thread();profile_thread().enabled=false;auto& totals=profile_totals();
 std::vector<std::pair<std::string,ProfileStat>> sorted(totals.rows.begin(),totals.rows.end());
 std::sort(sorted.begin(),sorted.end(),[](const auto& a,const auto& b){return a.second.exclusive_ns>b.second.exclusive_ns;});
 Args rows;size_t bytes=0;uint64_t omitted=totals.omitted_calls;
 for(const auto& pair:sorted){V row=make_dict({{"name",pair.first},{"calls",int64_t(pair.second.calls)},{"inclusive_ns",int64_t(pair.second.inclusive_ns)},{"exclusive_ns",int64_t(pair.second.exclusive_ns)}});auto encoded=json_dump(row);
  if(bytes+encoded.size()>550000){omitted+=pair.second.calls;continue;}bytes+=encoded.size();rows.push_back(std::move(row));}
 std::cerr<<json_dump(make_dict({{"type","native_profile"},{"rows",make_list(std::move(rows))},{"omitted_calls",int64_t(omitted)},{"detail_truncated",omitted!=0}}))<<std::endl;
}
// The static CRT's pow can differ from the Windows reference runtime by one
// binary64 ULP. Resolve the operating-system UCRT implementation explicitly;
// the calculation engine still has no Python or redistributable C++ dependency.
double reference_pow(double x,double y){
#ifdef _WIN32
 using Pow=double (__cdecl *)(double,double);
 static Pow function=[]()->Pow{
  HMODULE module=LoadLibraryExW(L"ucrtbase.dll",nullptr,LOAD_LIBRARY_SEARCH_SYSTEM32);
  if(!module)throw PyError("RuntimeError","Cannot load the operating-system UCRT for native pow");
  FARPROC address=GetProcAddress(module,"pow");
  if(!address){FreeLibrary(module);throw PyError("RuntimeError","The operating-system UCRT does not export native pow");}
  // Keep the OS module loaded for the lifetime of this cached function pointer.
  return reinterpret_cast<Pow>(address);
 }();
 return function(x,y);
#else
 return std::pow(x,y);
#endif
}
namespace {
[[noreturn]] void fail(const std::string& kind,const std::string& msg){throw PyError(kind,msg);}
bool numeric(V v){auto k=v.kind();return k==Kind::Bool||k==Kind::Int||k==Kind::Float;}
bool integral(V v){return v.kind()==Kind::Int||v.kind()==Kind::Bool;}
bool sequence(V v){return v.kind()==Kind::List||v.kind()==Kind::Tuple;}
V node(Kind k){return V(std::make_shared<Node>(k));}
size_t length(V v){switch(v.kind()){
case Kind::String:return v.as_string().size();case Kind::Bytes:case Kind::ByteArray:return v.node()->text.size();
case Kind::List:case Kind::Tuple:case Kind::Set:return v.node()->seq.size();
case Kind::Dict:return v.node()->attrs?v.node()->attrs->values.size():v.node()->entries.size();
default:fail("TypeError","object of type '"+type_name(v)+"' has no len()");}}
int64_t index(V v,size_t n){int64_t i=v.as_int();if(i<0)i+=int64_t(n);if(i<0||uint64_t(i)>=n)fail("IndexError","index out of range");return i;}
size_t entry(V d,V key){auto p=d.node();if(key.kind()==Kind::String){auto it=p->string_index.find(key.as_string());return it==p->string_index.end()?size_t(-1):it->second;}
for(size_t i=0;i<p->entries.size();++i)if(equal(p->entries[i].first,key))return i;return size_t(-1);}
void reindex(V d){auto p=d.node();p->string_index.clear();for(size_t i=0;i<p->entries.size();++i)if(p->entries[i].first.kind()==Kind::String)p->string_index[p->entries[i].first.as_string()]=i;}
Args keys(V d){Args out;auto p=d.node();if(p->attrs){for(auto& kv:p->attrs->values)out.emplace_back(kv.first);}else for(auto& kv:p->entries)out.push_back(kv.first);return out;}
V class_lookup(V cls,const std::string& name){if(cls.kind()!=Kind::Class)return {};auto p=cls.node();auto it=p->attrs->values.find(name);if(it!=p->attrs->values.end())return it->second;
for(auto& b:p->bases){V v=class_lookup(b,name);if(!v.is_none())return v;}return {};}
bool subclass(V cls,V base){if(identical(cls,base))return true;if(cls.kind()!=Kind::Class)return false;for(auto& b:cls.node()->bases)if(subclass(b,base))return true;return false;}
V bound(V f,V self){return builtin(f.kind()==Kind::Function?f.node()->text:"method",[f,self](const Args& a,const Kw& k){Args x;x.reserve(a.size()+1);x.push_back(self);x.insert(x.end(),a.begin(),a.end());return call(f,x,k);});}
V method(V v,const std::string& name){V f=class_lookup(v.kind()==Kind::Class?v:v.node()->cls,name);if(f.is_none())return {};if(f.kind()!=Kind::Function)return f;if(f.node()->class_method)return bound(f,v.kind()==Kind::Class?v:v.node()->cls);return v.kind()==Kind::Class?f:bound(f,v);}
void arity(const Args& a,size_t low,size_t high,const std::string& name){if(a.size()<low||a.size()>high)fail("TypeError",name+" received incorrect number of arguments");}
V kwget(const Kw& k,const std::string& name,V fallback={}){for(auto& x:k)if(x.first==name)return x.second;return fallback;}
std::string floats(double d,bool representation){if(std::isnan(d))return "nan";if(std::isinf(d))return d<0?"-inf":"inf";if(d==0)return std::signbit(d)?"-0.0":"0.0";
char buffer[128];auto result=std::to_chars(buffer,buffer+sizeof(buffer),d,std::chars_format::scientific);if(result.ec!=std::errc())fail("RuntimeError","floating conversion failed");std::string sci(buffer,result.ptr);size_t ep=sci.find('e');int exponent=std::stoi(sci.substr(ep+1));std::string sign=d<0?"-":"",digits=sci.substr(d<0?1:0,ep-(d<0?1:0));digits.erase(std::remove(digits.begin(),digits.end(),'.'),digits.end());std::string s;
if(exponent>=-4&&exponent<16){int point=exponent+1;if(point<=0)s=sign+"0."+std::string(size_t(-point),'0')+digits;else if(size_t(point)>=digits.size())s=sign+digits+std::string(size_t(point)-digits.size(),'0')+(representation?".0":"");else s=sign+digits.substr(0,size_t(point))+"."+digits.substr(size_t(point));}
else{s=sign+digits.substr(0,1);if(digits.size()>1)s+="."+digits.substr(1);s+="e";s+=exponent<0?"-":"+";int mag=std::abs(exponent);if(mag<10)s+="0";s+=std::to_string(mag);}return s;}
std::string escape(const std::string& s,char quote){std::string out(1,quote);const char* hex="0123456789abcdef";for(unsigned char c:s){switch(c){case '\\':out+="\\\\";break;case '\n':out+="\\n";break;case '\r':out+="\\r";break;case '\t':out+="\\t";break;case '\b':out+="\\b";break;case '\f':out+="\\f";break;default:if(c==quote){out+='\\';out+=c;}else if(c<32){out+="\\u00";out+=hex[c>>4];out+=hex[c&15];}else out+=c;}}out+=quote;return out;}
struct Slice {int64_t begin,end,step;};
Slice indices(size_t n,V start,V stop,V step){int64_t nn=int64_t(n),s=step.is_none()?1:step.as_int();if(!s)fail("ValueError","slice step cannot be zero");
auto fix=[nn,s](V x,bool first){if(x.is_none())return s>0?(first?int64_t(0):nn):(first?nn-1:int64_t(-1));int64_t i=x.as_int();if(i<0)i+=nn;return s>0?std::max<int64_t>(0,std::min(nn,i)):std::max<int64_t>(-1,std::min(nn-1,i));};return {fix(start,true),fix(stop,false),s};}
Args sorted_values(V values,V key,V reverse){Args a=iter(values);std::vector<std::pair<V,V>> keyed;keyed.reserve(a.size());for(auto& x:a)keyed.emplace_back(key.is_none()?x:call(key,{x}),x);
bool rev=truth(reverse);std::stable_sort(keyed.begin(),keyed.end(),[rev](auto& x,auto& y){return rev?compare("Lt",y.first,x.first):compare("Lt",x.first,y.first);});a.clear();for(auto& x:keyed)a.push_back(x.second);return a;}
}

PyError::PyError(std::string k,std::string m):std::runtime_error(m),kind(std::move(k)){}
Kind V::kind()const{switch(data.index()){case 0:return Kind::None;case 1:return Kind::Bool;case 2:return Kind::Int;case 3:return Kind::Float;case 4:return Kind::String;default:return std::get<std::shared_ptr<Node>>(data)->kind;}}
double V::as_double()const{if(auto p=std::get_if<double>(&data))return *p;if(auto p=std::get_if<int64_t>(&data))return double(*p);if(auto p=std::get_if<bool>(&data))return *p?1.:0.;fail("TypeError","expected a number, received "+type_name(*this));}
int64_t V::as_int()const{if(auto p=std::get_if<int64_t>(&data))return *p;if(auto p=std::get_if<bool>(&data))return *p?1:0;fail("TypeError","expected an integer, received "+type_name(*this));}
const std::string& V::as_string()const{if(auto p=std::get_if<std::string>(&data))return *p;if(kind()==Kind::Bytes||kind()==Kind::ByteArray)return node()->text;fail("TypeError","expected a string, received "+type_name(*this));}
const std::shared_ptr<Node>& V::node()const{if(auto p=std::get_if<std::shared_ptr<Node>>(&data))return *p;fail("TypeError","expected an object, received "+type_name(*this));}
E child(E parent){auto e=std::make_shared<Env>();e->parent=std::move(parent);return e;}
E root_env(E e){while(e&&e->parent&&e->parent!=builtins())e=e->parent;return e;}
V get(const E& e,const std::string& name){for(auto p=e.get();p;p=p->parent.get()){auto it=p->values.find(name);if(it!=p->values.end())return it->second;}auto b=builtins();auto it=b->values.find(name);if(it!=b->values.end())return it->second;fail("NameError","name '"+name+"' is not defined");}
void set(const E& e,const std::string& name,V value){e->values[name]=std::move(value);}
void set_outer(E e,const std::string& name,V value){for(auto p=e->parent;p;p=p->parent){auto it=p->values.find(name);if(it!=p->values.end()){it->second=std::move(value);return;}}fail("NameError","nonlocal name '"+name+"' is not defined");}
void del(E e,const std::string& name){if(!e->values.erase(name))fail("NameError","name '"+name+"' is not defined");}
namespace {std::vector<std::weak_ptr<Env>>& captured_frames(){static thread_local std::vector<std::weak_ptr<Env>> frames;return frames;}}
FrameGuard::~FrameGuard() = default;
InvocationGuard::InvocationGuard():start(captured_frames().size()){}
InvocationGuard::~InvocationGuard(){auto& frames=captured_frames();for(size_t i=start;i<frames.size();++i)if(auto e=frames[i].lock())e->values.clear();frames.resize(start);}
void track_capture(E env){for(auto e=env;e;e=e->parent)if(e->function_frame&&!e->closure_tracked){e->closure_tracked=true;captured_frames().push_back(e);}}
void bind(E e,const Args& a,const Kw& kw,const std::vector<std::string>& pos,int posonly,const std::vector<std::string>& ko,const Kw& defaults,const std::string& vararg,const std::string& kwarg){
if(a.size()>pos.size()&&vararg.empty())fail("TypeError","too many positional arguments");for(size_t i=0;i<std::min(a.size(),pos.size());++i)set(e,pos[i],a[i]);
if(!vararg.empty())set(e,vararg,make_tuple(a.size()>pos.size()?Args(a.begin()+pos.size(),a.end()):Args{}));V extra;if(!kwarg.empty())extra=make_dict();
std::unordered_set<std::string> seen_keywords;for(auto& pair:kw){if(!seen_keywords.insert(pair.first).second)fail("TypeError","multiple keyword values for '"+pair.first+"'");auto it=std::find(pos.begin(),pos.end(),pair.first);bool valid=it!=pos.end()&&std::distance(pos.begin(),it)>=posonly;valid=valid||std::find(ko.begin(),ko.end(),pair.first)!=ko.end();
if(!valid){if(kwarg.empty())fail("TypeError","unexpected keyword argument '"+pair.first+"'");dict_set(extra,pair.first,pair.second);continue;}if(e->values.count(pair.first))fail("TypeError","multiple values for argument '"+pair.first+"'");set(e,pair.first,pair.second);}
for(auto& x:defaults)if(!e->values.count(x.first))set(e,x.first,x.second);for(auto& n:pos)if(!e->values.count(n))fail("TypeError","missing required argument '"+n+"'");for(auto& n:ko)if(!e->values.count(n))fail("TypeError","missing required keyword argument '"+n+"'");if(!kwarg.empty())set(e,kwarg,extra);}
V builtin(const std::string& name,Builtin f){V v=node(Kind::Function);v.node()->text=name;v.node()->builtin=std::move(f);return v;}
V make_function(const std::string& name,E env,Fn fn){track_capture(env);V v=node(Kind::Function);v.node()->text=name;v.node()->closure=std::move(env);v.node()->fn=std::move(fn);return v;}
V make_class(const std::string& name,const Args& bases,E attrs){V v=node(Kind::Class);auto p=v.node();p->text=name;p->bases=bases;p->attrs=std::move(attrs);set(p->attrs,"__name__",name);return v;}
V make_module(const std::string& name,E env){V v=node(Kind::Module);v.node()->text=name;v.node()->attrs=std::move(env);return v;}
V make_list(Args a){V v=node(Kind::List);v.node()->seq=std::move(a);return v;}
V make_tuple(Args a){V v=node(Kind::Tuple);v.node()->seq=std::move(a);return v;}
V make_set(Args a){V v=node(Kind::Set);for(auto& x:a)if(!contains(v,x))v.node()->seq.push_back(x);return v;}
V make_dict(Kw a){V v=node(Kind::Dict);for(auto& x:a)dict_set(v,x.first,x.second);return v;}
V make_bytes(std::string s){V v=node(Kind::Bytes);v.node()->text=std::move(s);return v;}
V make_generator(std::function<bool(V&)> f){V v=node(Kind::Generator);v.node()->advance=std::move(f);return v;}
V make_generator(std::function<V()> f){return make_generator([f=std::move(f)](V& out){try{out=f();return true;}catch(const PyError& e){if(e.kind=="StopIteration")return false;throw;}});}
bool try_next(V v,V& out){if(v.kind()!=Kind::Generator)fail("TypeError","object is not an iterator");return v.node()->advance(out);}
V next(V v){V out;if(!try_next(v,out))fail("StopIteration","");return out;}
V iterator(V v){if(v.kind()==Kind::Generator)return v;auto pos=std::make_shared<size_t>(0);if(v.kind()==Kind::List||v.kind()==Kind::Tuple||v.kind()==Kind::Set)return make_generator([v,pos](V& out){if(*pos>=v.node()->seq.size())return false;out=v.node()->seq[(*pos)++];return true;});if(v.kind()==Kind::Instance){V m=method(v,"__iter__");if(!m.is_none())return call(m);}auto values=std::make_shared<Args>(iter(v));return make_generator([values,pos](V& out){if(*pos>=values->size())return false;out=(*values)[(*pos)++];return true;});}
[[noreturn]] void rethrow_call(const PyError& e,const std::string& name){if(e.kind=="StopIteration")throw e;std::string message=e.what();if(message.size()<16000)message+="\n  at "+name;PyError contextual(e.kind,message);contextual.value=e.value;throw contextual;}
V call(V f,const Args& a,const Kw& k){if(f.kind()==Kind::Function){auto p=f.node();ProfileScope profile(*p);try{if(p->unary_fast&&k.empty()&&a.size()==1)return p->unary_fast(a[0]);if(p->binary_fast&&k.empty()&&a.size()==2)return p->binary_fast(a[0],a[1]);if(p->builtin)return p->builtin(a,k);E frame=child(p->closure);frame->function_frame=true;FrameGuard guard(frame);return p->fn(frame,a,k);}catch(const PyError& e){rethrow_call(e,p->text);}}
if(f.kind()==Kind::Class){V v=node(Kind::Instance);v.node()->cls=f;v.node()->attrs=child();V init=method(v,"__init__");if(!init.is_none())call(init,a,k);else if(!a.empty()||!k.empty())fail("TypeError",f.node()->text+" takes no arguments");return v;}
if(f.kind()==Kind::Instance){V m=method(f,"__call__");if(!m.is_none())return call(m,a,k);}fail("TypeError","'"+type_name(f)+"' object is not callable");}
bool truth(V v){switch(v.kind()){case Kind::None:return false;case Kind::Bool:return std::get<bool>(v.data);case Kind::Int:return v.as_int()!=0;case Kind::Float:return v.as_double()!=0.;
case Kind::String:case Kind::Bytes:case Kind::ByteArray:case Kind::List:case Kind::Tuple:case Kind::Set:case Kind::Dict:return length(v)!=0;
case Kind::Instance:{V m=method(v,"__bool__");if(!m.is_none())return truth(call(m));m=method(v,"__len__");return m.is_none()?true:truth(call(m));}default:return true;}}
bool identical(V a,V b){if(a.kind()==Kind::None||a.kind()==Kind::Bool)return a.data==b.data;auto pa=std::get_if<std::shared_ptr<Node>>(&a.data),pb=std::get_if<std::shared_ptr<Node>>(&b.data);if(pa&&pb)return *pa==*pb;return false;}
bool equal(V a,V b){if(numeric(a)&&numeric(b)){if(integral(a)&&integral(b))return a.as_int()==b.as_int();return a.as_double()==b.as_double();}if(a.kind()!=b.kind())return false;if(a.is_none())return true;
if(a.kind()==Kind::String)return a.as_string()==b.as_string();if(identical(a,b))return true;switch(a.kind()){
case Kind::Bytes:case Kind::ByteArray:return a.node()->text==b.node()->text;case Kind::List:case Kind::Tuple:{auto& x=a.node()->seq;auto& y=b.node()->seq;if(x.size()!=y.size())return false;for(size_t i=0;i<x.size();++i)if(!equal(x[i],y[i]))return false;return true;}
case Kind::Dict:if(length(a)!=length(b))return false;for(auto& key:keys(a))if(!contains(b,key)||!equal(item(a,key),item(b,key)))return false;return true;
case Kind::Set:if(length(a)!=length(b))return false;for(auto& x:a.node()->seq)if(!contains(b,x))return false;return true;
case Kind::Instance:{V m=method(a,"__eq__");return m.is_none()?false:truth(call(m,{b}));}default:return false;}}
bool contains(V v,V key){switch(v.kind()){case Kind::Dict:{auto p=v.node();return p->attrs?key.kind()==Kind::String&&p->attrs->values.count(key.as_string()):entry(v,key)!=size_t(-1);}case Kind::String:if(key.kind()!=Kind::String)fail("TypeError","string membership requires string");return v.as_string().find(key.as_string())!=std::string::npos;
case Kind::Bytes:case Kind::ByteArray:if(key.kind()==Kind::Bytes||key.kind()==Kind::ByteArray)return v.node()->text.find(key.node()->text)!=std::string::npos;if(integral(key))return v.node()->text.find(char(key.as_int()))!=std::string::npos;fail("TypeError","invalid bytes membership");
case Kind::List:case Kind::Tuple:case Kind::Set:for(auto& x:v.node()->seq)if(equal(x,key))return true;return false;
case Kind::Generator:{V value;while(try_next(v,value))if(equal(value,key))return true;return false;}default:fail("TypeError","object is not iterable");}}
void dict_set(V d,V key,V value){if(d.kind()!=Kind::Dict)fail("TypeError","expected dict");auto p=d.node();if(p->attrs){p->attrs->values[key.as_string()]=std::move(value);return;}size_t i=entry(d,key);if(i!=size_t(-1)){p->entries[i].second=std::move(value);return;}if(key.kind()==Kind::List||key.kind()==Kind::Dict||key.kind()==Kind::Set)fail("TypeError","unhashable dictionary key");if(key.kind()==Kind::String)p->string_index[key.as_string()]=p->entries.size();p->entries.emplace_back(std::move(key),std::move(value));}
V dict_get(V d,V key,V fallback){if(!contains(d,key))return fallback;return item(d,key);}
void dict_merge(V d,V other){if(other.kind()==Kind::Dict){for(auto& k:keys(other))dict_set(d,k,item(other,k));return;}for(auto& pair:iter(other)){Args p=iter(pair);if(p.size()!=2)fail("ValueError","dictionary update element needs length 2");dict_set(d,p[0],p[1]);}}
V item(V v,V key){switch(v.kind()){case Kind::Dict:{auto p=v.node();if(p->attrs){auto it=p->attrs->values.find(key.as_string());if(it!=p->attrs->values.end())return it->second;}else{size_t i=entry(v,key);if(i!=size_t(-1))return p->entries[i].second;}fail("KeyError",repr(key));}
case Kind::List:case Kind::Tuple:return v.node()->seq[size_t(index(key,length(v)))];case Kind::String:return std::string(1,v.as_string()[size_t(index(key,length(v)))]);case Kind::Bytes:case Kind::ByteArray:return int((unsigned char)v.node()->text[size_t(index(key,length(v)))]);
case Kind::Instance:{V m=method(v,"__getitem__");if(!m.is_none())return call(m,{key});}default:fail("TypeError","'"+type_name(v)+"' object is not subscriptable");}}
void setitem(V v,V key,V value){if(v.kind()==Kind::Dict){dict_set(v,key,std::move(value));return;}if(v.kind()==Kind::List){v.node()->seq[size_t(index(key,length(v)))]=std::move(value);return;}if(v.kind()==Kind::ByteArray){int64_t c=value.as_int();if(c<0||c>255)fail("ValueError","byte outside 0..255");v.node()->text[size_t(index(key,length(v)))]=char(c);return;}if(v.kind()==Kind::Instance){V m=method(v,"__setitem__");if(!m.is_none()){call(m,{key,value});return;}}fail("TypeError","object does not support item assignment");}
void delitem(V v,V key){if(v.kind()==Kind::Dict){auto p=v.node();if(p->attrs){if(!p->attrs->values.erase(key.as_string()))fail("KeyError",repr(key));return;}size_t i=entry(v,key);if(i==size_t(-1))fail("KeyError",repr(key));p->entries.erase(p->entries.begin()+i);reindex(v);return;}if(v.kind()==Kind::List){auto& a=v.node()->seq;a.erase(a.begin()+index(key,a.size()));return;}fail("TypeError","object does not support item deletion");}
V slice(V v,V start,V stop,V step){auto ix=indices(length(v),start,stop,step);Args out;std::string text;for(int64_t i=ix.begin;ix.step>0?i<ix.end:i>ix.end;i+=ix.step){if(sequence(v))out.push_back(v.node()->seq[size_t(i)]);else if(v.kind()==Kind::String)text+=v.as_string()[size_t(i)];else if(v.kind()==Kind::Bytes||v.kind()==Kind::ByteArray)text+=v.node()->text[size_t(i)];else fail("TypeError","object does not support slices");}if(v.kind()==Kind::String)return text;if(v.kind()==Kind::Bytes||v.kind()==Kind::ByteArray){V out=make_bytes(text);out.node()->kind=v.kind();return out;}return v.kind()==Kind::Tuple?make_tuple(std::move(out)):make_list(std::move(out));}
void setslice(V v,V start,V stop,V step,V values){if(v.kind()==Kind::ByteArray){Args replacement=iter(values);std::string bytes;for(auto& x:replacement){int64_t c=x.as_int();if(c<0||c>255)fail("ValueError","byte outside 0..255");bytes+=char(c);}auto ix=indices(length(v),start,stop,step);if(ix.step==1){v.node()->text.replace(size_t(ix.begin),size_t(std::max<int64_t>(0,ix.end-ix.begin)),bytes);return;}size_t n=0;for(int64_t i=ix.begin;ix.step>0?i<ix.end:i>ix.end;i+=ix.step)++n;if(n!=bytes.size())fail("ValueError","extended slice size mismatch");n=0;for(int64_t i=ix.begin;ix.step>0?i<ix.end:i>ix.end;i+=ix.step)v.node()->text[size_t(i)]=bytes[n++];return;}if(v.kind()!=Kind::List)fail("TypeError","slice assignment requires list");auto ix=indices(length(v),start,stop,step);Args repl=iter(values);auto& a=v.node()->seq;if(ix.step==1){auto last=std::max(ix.begin,ix.end);a.erase(a.begin()+ix.begin,a.begin()+last);a.insert(a.begin()+ix.begin,repl.begin(),repl.end());return;}std::vector<size_t> ids;for(int64_t i=ix.begin;ix.step>0?i<ix.end:i>ix.end;i+=ix.step)ids.push_back(size_t(i));if(ids.size()!=repl.size())fail("ValueError","extended slice size mismatch");for(size_t i=0;i<ids.size();++i)a[ids[i]]=repl[i];}
Args iter(V v){switch(v.kind()){case Kind::List:case Kind::Tuple:case Kind::Set:return v.node()->seq;case Kind::Dict:return keys(v);case Kind::String:{Args a;for(char c:v.as_string())a.emplace_back(std::string(1,c));return a;}case Kind::Bytes:case Kind::ByteArray:{Args a;for(unsigned char c:v.node()->text)a.emplace_back(int(c));return a;}case Kind::Generator:{Args a;V value;while(try_next(v,value))a.push_back(std::move(value));return a;}case Kind::Instance:{V m=method(v,"__iter__");if(!m.is_none())return iter(call(m));}default:fail("TypeError","'"+type_name(v)+"' object is not iterable");}}
bool compare(const std::string& op,V a,V b){if(op=="Eq"||op=="==")return equal(a,b);if(op=="NotEq"||op=="!=")return !equal(a,b);if(op=="Is"||op=="is")return identical(a,b);if(op=="IsNot"||op=="is not")return !identical(a,b);if(op=="In"||op=="in")return contains(b,a);if(op=="NotIn"||op=="not in")return !contains(b,a);
bool lt=false,eq=false,gt=false;if(numeric(a)&&numeric(b)){if(integral(a)&&integral(b)){lt=a.as_int()<b.as_int();eq=a.as_int()==b.as_int();gt=a.as_int()>b.as_int();}else{double x=a.as_double(),y=b.as_double();lt=x<y;eq=x==y;gt=x>y;}}
else if(a.kind()==Kind::String&&b.kind()==Kind::String){lt=a.as_string()<b.as_string();eq=a.as_string()==b.as_string();gt=a.as_string()>b.as_string();}
else if(a.kind()==b.kind()&&sequence(a)){auto x=iter(a),y=iter(b);size_t i=0;while(i<x.size()&&i<y.size()&&equal(x[i],y[i]))++i;if(i==x.size()||i==y.size()){lt=x.size()<y.size();eq=x.size()==y.size();gt=x.size()>y.size();}else{lt=compare("Lt",x[i],y[i]);gt=compare("Gt",x[i],y[i]);}}
else if(a.kind()==Kind::Set&&b.kind()==Kind::Set){bool sub=true;for(auto& x:a.node()->seq)sub=sub&&contains(b,x);eq=equal(a,b);lt=sub&&!eq;bool sup=true;for(auto& x:b.node()->seq)sup=sup&&contains(a,x);gt=sup&&!eq;}
else fail("TypeError","unsupported ordered comparison between "+type_name(a)+" and "+type_name(b));
if(op=="Lt"||op=="<")return lt;if(op=="LtE"||op=="<=")return lt||eq;if(op=="Gt"||op==">")return gt;if(op=="GtE"||op==">=")return gt||eq;fail("NotImplementedError","comparison "+op);}
V binary(const std::string& op,V a,V b){if(a.kind()==Kind::Instance){std::string special=op=="Div"?"__truediv__":op=="Add"?"__add__":op=="Sub"?"__sub__":op=="Mult"?"__mul__":"";if(!special.empty()){V m=method(a,special);if(!m.is_none())return call(m,{b});}}
if(op=="Add"||op=="+"){if(a.kind()==Kind::String&&b.kind()==Kind::String)return a.as_string()+b.as_string();if(a.kind()==Kind::Bytes&&b.kind()==Kind::Bytes)return make_bytes(a.node()->text+b.node()->text);if(sequence(a)&&a.kind()==b.kind()){Args v=iter(a),w=iter(b);v.insert(v.end(),w.begin(),w.end());return a.kind()==Kind::List?make_list(std::move(v)):make_tuple(std::move(v));}}
if(op=="Mult"||op=="*"){V seq=a,num=b;if(integral(a)){seq=b;num=a;}if(integral(num)&&(sequence(seq)||seq.kind()==Kind::String||seq.kind()==Kind::Bytes)){int64_t n=std::max<int64_t>(0,num.as_int());if(sequence(seq)){Args out,s=iter(seq);if(n&&uint64_t(n)>std::numeric_limits<size_t>::max()/std::max<size_t>(1,s.size()))fail("OverflowError","repeated sequence too large");for(int64_t i=0;i<n;++i)out.insert(out.end(),s.begin(),s.end());return seq.kind()==Kind::List?make_list(std::move(out)):make_tuple(std::move(out));}std::string out,s=seq.as_string();for(int64_t i=0;i<n;++i)out+=s;return seq.kind()==Kind::Bytes?make_bytes(std::move(out)):V(std::move(out));}}
if((op=="BitOr"||op=="|")&&a.kind()==Kind::Dict&&b.kind()==Kind::Dict){V out=make_dict();dict_merge(out,a);dict_merge(out,b);return out;}
if(a.kind()==Kind::Set&&b.kind()==Kind::Set){V out=make_set();for(auto& x:a.node()->seq)if(op=="BitOr"||(op=="BitAnd"&&contains(b,x))||(op=="Sub"&&!contains(b,x))||(op=="BitXor"&&!contains(b,x)))out.node()->seq.push_back(x);if(op=="BitOr"||op=="BitXor")for(auto& x:b.node()->seq)if(!contains(a,x))out.node()->seq.push_back(x);return out;}
if(!numeric(a)||!numeric(b))fail("TypeError","unsupported operands for "+op+": "+type_name(a)+", "+type_name(b));bool ints=integral(a)&&integral(b);double x=a.as_double(),y=b.as_double();
if(op=="Add"||op=="+")return ints?V(int64_t(uint64_t(a.as_int())+uint64_t(b.as_int()))):V(x+y);
if(op=="Sub"||op=="-")return ints?V(int64_t(uint64_t(a.as_int())-uint64_t(b.as_int()))):V(x-y);
if(op=="Mult"||op=="*")return ints?V(int64_t(uint64_t(a.as_int())*uint64_t(b.as_int()))):V(x*y);
if(op=="Div"||op=="/"){if(y==0)fail("ZeroDivisionError","division by zero");return x/y;}
if(op=="FloorDiv"||op=="//"){if(y==0)fail("ZeroDivisionError","division by zero");return ints?V(int64_t(std::floor(x/y))):V(std::floor(x/y));}
if(op=="Mod"||op=="%"){if(y==0)fail("ZeroDivisionError","modulo by zero");if(ints){int64_t r=a.as_int()%b.as_int();if(r&&((r<0)!=(b.as_int()<0)))r+=b.as_int();return r;}double r=std::fmod(x,y);if(r&&((r<0)!=(y<0)))r+=y;if(r==0)r=std::copysign(0.,y);return r;}
if(op=="Pow"||op=="**"){if(x==0&&y<0)fail("ZeroDivisionError","zero cannot be raised to a negative power");if(ints&&b.as_int()>=0){uint64_t result=1,base=uint64_t(a.as_int());int64_t n=b.as_int();while(n){if(n&1)result*=base;n>>=1;if(n)base*=base;}return int64_t(result);}return reference_pow(x,y);}
int64_t ix=a.as_int(),iy=b.as_int();if(op=="BitAnd"||op=="&")return ix&iy;if(op=="BitOr"||op=="|")return ix|iy;if(op=="BitXor"||op=="^")return ix^iy;if(op=="LShift"||op=="<<"){if(iy<0||iy>=64)fail("OverflowError","native integer shift outside 0..63");return int64_t(uint64_t(ix)<<iy);}if(op=="RShift"||op==">>"){if(iy<0)fail("ValueError","negative shift count");return iy>=64?V(ix<0?-1:0):V(ix>>iy);}fail("NotImplementedError","binary operator "+op);}
V inplace(const std::string& op,V a,V b){if(a.kind()==Kind::List&&(op=="Add"||op=="+")){Args rest=iter(b);a.node()->seq.insert(a.node()->seq.end(),rest.begin(),rest.end());return a;}if(a.kind()==Kind::List&&(op=="Mult"||op=="*")){a.node()->seq=binary(op,a,b).node()->seq;return a;}if(a.kind()==Kind::Dict&&(op=="BitOr"||op=="|")){dict_merge(a,b);return a;}return binary(op,a,b);}
V unary(const std::string& op,V v){if(op=="Not"||op=="not")return !truth(v);if(op=="UAdd"||op=="+")return integral(v)?V(v.as_int()):V(v.as_double());if(op=="USub"||op=="-")return integral(v)?V(int64_t(0-uint64_t(v.as_int()))):V(-v.as_double());if(op=="Invert"||op=="~")return ~v.as_int();fail("NotImplementedError","unary operator "+op);}
std::string type_name(V v){switch(v.kind()){case Kind::None:return "NoneType";case Kind::Bool:return "bool";case Kind::Int:return "int";case Kind::Float:return "float";case Kind::String:return "str";case Kind::Bytes:return "bytes";case Kind::ByteArray:return "bytearray";case Kind::List:return "list";case Kind::Tuple:return "tuple";case Kind::Dict:return "dict";case Kind::Set:return "set";case Kind::Function:return "function";case Kind::Class:return "type";case Kind::Module:return "module";case Kind::Generator:return "generator";case Kind::Instance:return v.node()->cls.node()->text;case Kind::Exception:return v.node()->text;default:return "object";}}
std::string str(V v){if(v.kind()==Kind::String)return v.as_string();if(v.kind()==Kind::Instance){V m=method(v,"__str__");if(!m.is_none())return call(m).as_string();}if(v.kind()==Kind::Exception)return v.node()->attrs?str(get(v.node()->attrs,"message")):v.node()->text;return repr(v);}
std::string repr(V v){switch(v.kind()){case Kind::None:return "None";case Kind::Bool:return truth(v)?"True":"False";case Kind::Int:return std::to_string(v.as_int());case Kind::Float:return floats(v.as_double(),true);case Kind::String:return escape(v.as_string(),'\'');case Kind::Bytes:return "b"+escape(v.node()->text,'\'');case Kind::ByteArray:return "bytearray(b"+escape(v.node()->text,'\'')+")";
case Kind::List:case Kind::Tuple:case Kind::Set:{std::string s=v.kind()==Kind::List?"[":v.kind()==Kind::Tuple?"(":"{";auto& a=v.node()->seq;for(size_t i=0;i<a.size();++i){if(i)s+=", ";s+=repr(a[i]);}if(v.kind()==Kind::Tuple&&a.size()==1)s+=",";s+=v.kind()==Kind::List?"]":v.kind()==Kind::Tuple?")":"}";return v.kind()==Kind::Set&&a.empty()?"set()":s;}
case Kind::Dict:{std::string s="{";bool first=true;for(auto& k:keys(v)){if(!first)s+=", ";first=false;s+=repr(k)+": "+repr(item(v,k));}return s+"}";}
case Kind::Class:return "<class '"+v.node()->text+"'>";case Kind::Function:return "<function "+v.node()->text+">";case Kind::Instance:{V m=method(v,"__repr__");if(!m.is_none())return call(m).as_string();return "<"+type_name(v)+" object>";}case Kind::Exception:return v.node()->text+"("+repr(get(v.node()->attrs,"message"))+")";default:return "<"+type_name(v)+">";}}

V attr(V v,const std::string& name){auto k=v.kind();if(k==Kind::Instance||k==Kind::Class||k==Kind::Module||k==Kind::Exception){auto p=v.node();if(name=="__dict__"){V d=make_dict();d.node()->attrs=p->attrs;return d;}if(name=="__class__"&&k==Kind::Instance)return p->cls;if(k==Kind::Class&&name=="__base__")return p->bases.empty()?get(builtins(),"object"):p->bases.front();if(p->attrs){auto it=p->attrs->values.find(name);if(it!=p->attrs->values.end()){V x=it->second;if(k==Kind::Class&&x.kind()==Kind::Function&&x.node()->class_method)return bound(x,v);return x;}}
if(k==Kind::Class||k==Kind::Instance){V m=method(v,name);if(!m.is_none())return m;if(name=="__new__")return builtin("__new__",[](const Args& a,const Kw&){arity(a,1,1,"__new__");V x=node(Kind::Instance);x.node()->cls=a[0];x.node()->attrs=child();return x;});if(k==Kind::Instance){V fallback=method(v,"__getattr__");if(!fallback.is_none())return call(fallback,{name});}}
fail("AttributeError",type_name(v)+" has no attribute '"+name+"'");}
if(k==Kind::Super){auto p=v.node();for(auto& base:p->cls.node()->bases){V m=class_lookup(base,name);if(!m.is_none())return bound(m,p->seq[0]);}fail("AttributeError","super has no attribute "+name);}
if(k==Kind::Function&&name=="__name__")return v.node()->text;
if(k==Kind::Function&&name=="fromkeys"&&v.node()->text=="dict")return builtin("dict.fromkeys",[](const Args& a,const Kw&){arity(a,1,2,"dict.fromkeys");V out=make_dict(),value=a.size()>1?a[1]:V();for(auto& key:iter(a[0]))dict_set(out,key,value);return out;});
if((k==Kind::Int||k==Kind::Bool)&&name=="bit_length")return builtin("int.bit_length",[v](const Args& a,const Kw&){arity(a,0,0,"int.bit_length");int64_t value=v.as_int();uint64_t n=value<0?uint64_t(0)-uint64_t(value):uint64_t(value);int bits=0;while(n){++bits;n>>=1;}return V(bits);});
if(k==Kind::Function&&name=="fromhex"&&v.node()->text=="bytes")return builtin("bytes.fromhex",[](const Args& a,const Kw&){arity(a,1,1,"bytes.fromhex");auto s=a[0].as_string();std::string out;size_t i=0;auto nibble=[](char c){if(c>='0'&&c<='9')return int(c-'0');if(c>='a'&&c<='f')return int(c-'a'+10);if(c>='A'&&c<='F')return int(c-'A'+10);return -1;};while(i<s.size()){while(i<s.size()&&std::isspace((unsigned char)s[i]))++i;if(i==s.size())break;if(i+1>=s.size()||nibble(s[i])<0||nibble(s[i+1])<0)fail("ValueError","non-hexadecimal digit in bytes.fromhex");out+=char((nibble(s[i])<<4)|nibble(s[i+1]));i+=2;}return make_bytes(out);});
if(k==Kind::Function&&name=="fromhex"&&v.node()->text=="float")return builtin("float.fromhex",[](const Args& a,const Kw&){arity(a,1,1,"float.fromhex");std::string s=a[0].as_string();size_t first=s.find_first_not_of(" \t\r\n\v\f"),last=s.find_last_not_of(" \t\r\n\v\f");if(first==std::string::npos)fail("ValueError","invalid hexadecimal floating-point string");s=s.substr(first,last-first+1);size_t sign=(s[0]=='+'||s[0]=='-')?1:0;std::string lower=s;for(char& c:lower)c=char(std::tolower((unsigned char)c));if(lower.substr(sign)!="inf"&&lower.substr(sign)!="infinity"&&lower.substr(sign)!="nan"){if(lower.compare(sign,2,"0x")!=0)s.insert(sign,"0x");if(s.find_first_of("pP")==std::string::npos)s+="p0";}errno=0;char* end=nullptr;double d=std::strtod(s.c_str(),&end);if(end!=s.c_str()+s.size())fail("ValueError","invalid hexadecimal floating-point string");if(errno==ERANGE&&std::isinf(d))fail("OverflowError","hexadecimal value too large to represent");return V(d);});
if(k==Kind::Dict){if(name=="get")return builtin("dict.get",[v](const Args& a,const Kw&){arity(a,1,2,"get");return dict_get(v,a[0],a.size()>1?a[1]:V());});
if(name=="keys"||name=="values"||name=="items")return builtin("dict."+name,[v,name](const Args& a,const Kw&){arity(a,0,0,name);Args out;for(auto& key:keys(v))out.push_back(name=="keys"?key:name=="values"?item(v,key):make_tuple({key,item(v,key)}));return name=="keys"?make_set(std::move(out)):make_list(std::move(out));});
if(name=="update")return builtin("dict.update",[v](const Args& a,const Kw& kw){arity(a,0,1,"update");if(!a.empty())dict_merge(v,a[0]);for(auto& x:kw)dict_set(v,x.first,x.second);return V();});
if(name=="setdefault")return builtin("dict.setdefault",[v](const Args& a,const Kw&){arity(a,1,2,"setdefault");if(contains(v,a[0]))return item(v,a[0]);V x=a.size()>1?a[1]:V();dict_set(v,a[0],x);return x;});
if(name=="pop")return builtin("dict.pop",[v](const Args& a,const Kw&){arity(a,1,2,"pop");if(!contains(v,a[0])){if(a.size()>1)return a[1];fail("KeyError",repr(a[0]));}V x=item(v,a[0]);delitem(v,a[0]);return x;});
if(name=="copy")return builtin("dict.copy",[v](const Args& a,const Kw&){arity(a,0,0,"copy");V d=make_dict();dict_merge(d,v);return d;});
if(name=="clear")return builtin("dict.clear",[v](const Args& a,const Kw&){arity(a,0,0,"clear");if(v.node()->attrs)v.node()->attrs->values.clear();v.node()->entries.clear();v.node()->string_index.clear();return V();});}
if(k==Kind::List||k==Kind::Tuple||k==Kind::Set){if(name=="copy")return builtin("copy",[v](const Args&,const Kw&){return v.kind()==Kind::Set?make_set(iter(v)):v.kind()==Kind::Tuple?make_tuple(iter(v)):make_list(iter(v));});
if(name=="count")return builtin("count",[v](const Args& a,const Kw&){arity(a,1,1,"count");int64_t n=0;for(auto& x:v.node()->seq)if(equal(x,a[0]))++n;return V(n);});
if(name=="index")return builtin("index",[v](const Args& a,const Kw&){arity(a,1,3,"index");auto s=indices(length(v),a.size()>1?a[1]:V(),a.size()>2?a[2]:V(),V(1));for(int64_t i=s.begin;i<s.end;++i)if(equal(v.node()->seq[size_t(i)],a[0]))return V(i);fail("ValueError","value not in sequence");});
if(k==Kind::List||k==Kind::Set){if(name=="append"||name=="add")return builtin(name,[v,k](const Args& a,const Kw&){arity(a,1,1,"append/add");if(k!=Kind::Set||!contains(v,a[0]))v.node()->seq.push_back(a[0]);return V();});
if(name=="extend"||name=="update")return builtin(name,[v,k](const Args& a,const Kw&){arity(a,1,1,"extend/update");for(auto& x:iter(a[0]))if(k!=Kind::Set||!contains(v,x))v.node()->seq.push_back(x);return V();});
if(name=="pop")return builtin("pop",[v](const Args& a,const Kw&){arity(a,0,1,"pop");auto& s=v.node()->seq;size_t i=size_t(index(a.empty()?V(-1):a[0],s.size()));V x=s[i];s.erase(s.begin()+i);return x;});
if(name=="remove"||name=="discard")return builtin(name,[v,name](const Args& a,const Kw&){arity(a,1,1,name);auto& s=v.node()->seq;for(auto it=s.begin();it!=s.end();++it)if(equal(*it,a[0])){s.erase(it);return V();}if(name=="remove")fail("ValueError","value not in collection");return V();});
if(name=="clear")return builtin("clear",[v](const Args&,const Kw&){v.node()->seq.clear();return V();});}
if(k==Kind::List){if(name=="insert")return builtin("insert",[v](const Args& a,const Kw&){arity(a,2,2,"insert");int64_t i=a[0].as_int(),n=int64_t(length(v));if(i<0)i+=n;i=std::max<int64_t>(0,std::min(n,i));v.node()->seq.insert(v.node()->seq.begin()+i,a[1]);return V();});
if(name=="reverse")return builtin("reverse",[v](const Args&,const Kw&){std::reverse(v.node()->seq.begin(),v.node()->seq.end());return V();});
if(name=="sort")return builtin("sort",[v](const Args& a,const Kw& k){arity(a,0,0,"sort");v.node()->seq=sorted_values(v,kwget(k,"key"),kwget(k,"reverse",false));return V();});}}
if(k==Kind::String||k==Kind::Bytes||k==Kind::ByteArray){if(name=="encode")return builtin("encode",[v](const Args& a,const Kw&){if(!a.empty()&&a[0].as_string()!="utf-8"&&a[0].as_string()!="ascii")fail("NotImplementedError","unsupported encoding");return make_bytes(v.as_string());});
if(name=="decode")return builtin("decode",[v](const Args& a,const Kw&){if(!a.empty()&&a[0].as_string()!="utf-8"&&a[0].as_string()!="ascii")fail("NotImplementedError","unsupported encoding");return V(v.as_string());});
if(name=="hex"&&(k==Kind::Bytes||k==Kind::ByteArray))return builtin("hex",[v](const Args&,const Kw&){std::string s;const char* h="0123456789abcdef";for(unsigned char c:v.as_string()){s+=h[c>>4];s+=h[c&15];}return V(s);});
if(name=="lower"||name=="upper")return builtin(name,[v,name](const Args& a,const Kw&){arity(a,0,0,name);std::string s=v.as_string();for(char& c:s)c=char(name=="lower"?std::tolower((unsigned char)c):std::toupper((unsigned char)c));return V(s);});
if(name=="startswith"||name=="endswith")return builtin(name,[v,name](const Args& a,const Kw&){arity(a,1,3,name);std::string s=slice(v,a.size()>1?a[1]:V(),a.size()>2?a[2]:V()).as_string();Args candidates=a[0].kind()==Kind::Tuple?iter(a[0]):Args{a[0]};for(auto& c:candidates){auto p=c.as_string();if(s.size()>=p.size()&&(name=="startswith"?s.compare(0,p.size(),p)==0:s.compare(s.size()-p.size(),p.size(),p)==0))return V(true);}return V(false);});
if(name=="removesuffix"||name=="removeprefix")return builtin(name,[v,name](const Args& a,const Kw&){arity(a,1,1,name);auto s=v.as_string(),p=a[0].as_string();if(!p.empty()&&s.size()>=p.size()){if(name=="removesuffix"&&s.compare(s.size()-p.size(),p.size(),p)==0)s.resize(s.size()-p.size());if(name=="removeprefix"&&s.compare(0,p.size(),p)==0)s.erase(0,p.size());}return V(s);});
if(name=="strip"||name=="lstrip"||name=="rstrip")return builtin(name,[v,name](const Args& a,const Kw&){arity(a,0,1,name);auto s=v.as_string();std::string chars=a.empty()||a[0].is_none()?" \t\r\n\v\f":a[0].as_string();size_t first=0,last=s.size();if(name!="rstrip")while(first<last&&chars.find(s[first])!=std::string::npos)++first;if(name!="lstrip")while(last>first&&chars.find(s[last-1])!=std::string::npos)--last;return V(s.substr(first,last-first));});
if(name=="join")return builtin("join",[v](const Args& a,const Kw&){arity(a,1,1,"join");std::string out,sep=v.as_string();bool first=true;for(auto& x:iter(a[0])){if(!first)out+=sep;first=false;out+=x.as_string();}return V(out);});
if(name=="split"||name=="rsplit")return builtin(name,[v,name](const Args& a,const Kw&){arity(a,0,2,name);std::string s=v.as_string();int64_t limit=a.size()>1?a[1].as_int():-1;Args out;if(a.empty()||a[0].is_none()){size_t i=0;while(i<s.size()){while(i<s.size()&&std::isspace((unsigned char)s[i]))++i;if(i==s.size())break;size_t j=i;if(limit>=0&&int64_t(out.size())==limit){out.emplace_back(s.substr(i));break;}while(j<s.size()&&!std::isspace((unsigned char)s[j]))++j;out.emplace_back(s.substr(i,j-i));i=j;}return make_list(out);}auto sep=a[0].as_string();if(sep.empty())fail("ValueError","empty separator");if(name=="split"){size_t i=0;while(limit<0||int64_t(out.size())<limit){size_t j=s.find(sep,i);if(j==std::string::npos)break;out.emplace_back(s.substr(i,j-i));i=j+sep.size();}out.emplace_back(s.substr(i));}else{size_t end=s.size();while(limit<0||int64_t(out.size())<limit){size_t j=end>=sep.size()?s.rfind(sep,end-sep.size()):std::string::npos;if(j==std::string::npos)break;out.emplace_back(s.substr(j+sep.size(),end-j-sep.size()));end=j;}out.emplace_back(s.substr(0,end));std::reverse(out.begin(),out.end());}return make_list(out);});
if(name=="replace")return builtin("replace",[v](const Args& a,const Kw&){arity(a,2,3,"replace");auto s=v.as_string(),from=a[0].as_string(),to=a[1].as_string();int64_t count=a.size()>2?a[2].as_int():-1;size_t i=0;int64_t n=0;if(from.empty())fail("NotImplementedError","empty substring replacement");while(count<0||n<count){i=s.find(from,i);if(i==std::string::npos)break;s.replace(i,from.size(),to);i+=to.size();++n;}return V(s);});
if(name=="find"||name=="rfind")return builtin(name,[v,name](const Args& a,const Kw&){arity(a,1,3,name);auto s=v.as_string();auto ix=indices(s.size(),a.size()>1?a[1]:V(),a.size()>2?a[2]:V(),V(1));auto sub=s.substr(size_t(ix.begin),size_t(std::max<int64_t>(0,ix.end-ix.begin)));size_t p=name=="find"?sub.find(a[0].as_string()):sub.rfind(a[0].as_string());return p==std::string::npos?V(-1):V(int64_t(p)+ix.begin);});}
fail("AttributeError",type_name(v)+" has no attribute '"+name+"'");}
void setattr(V v,const std::string& name,V value){if(v.kind()!=Kind::Instance&&v.kind()!=Kind::Class&&v.kind()!=Kind::Module&&v.kind()!=Kind::Exception)fail("AttributeError","object has no writable attributes");if(!v.node()->attrs)v.node()->attrs=child();set(v.node()->attrs,name,std::move(value));}
void delattr(V v,const std::string& name){if(!v.node()->attrs||!v.node()->attrs->values.erase(name))fail("AttributeError","missing attribute "+name);}
V classmethod(V f){f.node()->class_method=true;return f;}
V super_of(V cls,V self){V v=node(Kind::Super);v.node()->cls=cls;v.node()->seq={self};return v;}
V dataclass(V cls,const std::vector<std::string>& fields,const Kw& defaults,bool){V init=builtin(cls.node()->text+".__init__",[fields,defaults](const Args& args,const Kw& kwargs){if(args.empty())fail("TypeError","missing self");E e=child();bind(e,Args(args.begin()+1,args.end()),kwargs,fields,0,{},defaults);for(auto& n:fields)setattr(args[0],n,get(e,n));return V();});setattr(cls,"__init__",init);
setattr(cls,"__eq__",builtin("dataclass.__eq__",[fields](const Args& a,const Kw&){arity(a,2,2,"__eq__");if(a[1].kind()!=Kind::Instance||!identical(a[1].node()->cls,a[0].node()->cls))return V(false);for(auto& n:fields)if(!equal(attr(a[0],n),attr(a[1],n)))return V(false);return V(true);}));return cls;}
namespace {V clone(const V& v,std::unordered_map<const Node*,V>& memo){
 auto p=std::get_if<std::shared_ptr<Node>>(&v.data);if(!p)return v;
 const Node& source=**p;Kind k=source.kind;
 if(k==Kind::Function||k==Kind::Class||k==Kind::Module||k==Kind::Generator)return v;
 auto found=memo.find(p->get());if(found!=memo.end())return found->second;
 V out=node(k);memo.emplace(p->get(),out);Node& target=*out.node();target.text=source.text;target.cls=source.cls;
 target.seq.reserve(source.seq.size());for(const auto& value:source.seq)target.seq.push_back(clone(value,memo));
 if(k==Kind::Dict&&source.attrs){
  target.entries.reserve(source.attrs->values.size());target.string_index.reserve(source.attrs->values.size());
  for(const auto& pair:source.attrs->values)dict_set(out,pair.first,clone(pair.second,memo));
 }else{
  // Dictionary indices depend only on key order, which a deep copy preserves.
  // Copy them once rather than allocating a temporary key list and performing
  // two hash-table lookups for every field in every copied state.
  target.entries.reserve(source.entries.size());target.string_index=source.string_index;
  for(const auto& pair:source.entries)target.entries.emplace_back(clone(pair.first,memo),clone(pair.second,memo));
 }
 if(source.attrs&&k!=Kind::Dict){target.attrs=child();target.attrs->values.reserve(source.attrs->values.size());for(const auto& pair:source.attrs->values)set(target.attrs,pair.first,clone(pair.second,memo));}
 return out;
}}
V deepcopy(V v){std::unordered_map<const Node*,V> memo;return clone(v,memo);}
V exception_value(const PyError& e){if(!e.value.is_none())return e.value;V v=node(Kind::Exception);v.node()->text=e.kind;v.node()->attrs=child();set(v.node()->attrs,"message",std::string(e.what()));set(v.node()->attrs,"args",make_tuple({std::string(e.what())}));return v;}
bool exception_matches(const PyError& e,V cls){if(cls.kind()==Kind::Tuple){for(auto& x:iter(cls))if(exception_matches(e,x))return true;return false;}std::string n=cls.kind()==Kind::Class||cls.kind()==Kind::Function?cls.node()->text:str(cls);return n==e.kind||n=="Exception"||n=="BaseException"||(n=="ArithmeticError"&&(e.kind=="ZeroDivisionError"||e.kind=="OverflowError"))||(n=="LookupError"&&(e.kind=="KeyError"||e.kind=="IndexError"));}
[[noreturn]] void raise(V v){if(v.kind()==Kind::Class||v.kind()==Kind::Function)v=call(v);if(v.kind()==Kind::Exception){PyError e(v.node()->text,str(v));e.value=v;throw e;}fail("TypeError","exceptions must derive from BaseException");}

namespace {std::unordered_map<std::string,ModuleInit>& module_initializers(){static std::unordered_map<std::string,ModuleInit> m;return m;}std::unordered_map<std::string,V>& module_values(){static thread_local std::unordered_map<std::string,V> m;return m;}}
void register_module(const std::string& name,ModuleInit f){module_initializers()[name]=std::move(f);}
V import_module(const std::string& name){auto& values=module_values();auto& initializers=module_initializers();auto found=values.find(name);if(found!=values.end())return found->second;auto init=initializers.find(name);if(init==initializers.end())fail("ImportError","native module '"+name+"' is unavailable");E env=child(builtins());set(env,"__name__",name);V v=make_module(name,env);values[name]=v;try{init->second(env);}catch(...){env->values.clear();values.erase(name);throw;}return v;}
E module_env(const std::string& name){return import_module(name).node()->attrs;}
// Worker-thread teardown only: its native module graphs must not outlive the
// thread. Main-thread classes referenced by search jobs are not registered here.
void release_thread_runtime(){profile_merge_thread();auto& frames=captured_frames();for(auto& weak:frames)if(auto env=weak.lock())env->values.clear();frames.clear();auto& modules=module_values();for(auto& pair:modules)pair.second.node()->attrs->values.clear();modules.clear();builtins()->values.clear();}

E builtins(){static thread_local E b=child();static thread_local bool ready=false;if(ready)return b;ready=true;
auto put=[&](const std::string& name,Builtin f){set(b,name,builtin(name,std::move(f)));};
put("len",[](const Args& a,const Kw&){arity(a,1,1,"len");return V(int64_t(length(a[0])));});
put("bool",[](const Args& a,const Kw&){arity(a,0,1,"bool");return V(a.empty()?false:truth(a[0]));});
put("type",[](const Args& a,const Kw&){arity(a,1,1,"type");if(a[0].kind()==Kind::Instance)return a[0].node()->cls;std::string name=type_name(a[0]);auto b=builtins();auto it=b->values.find(name);if(it!=b->values.end())return it->second;fail("NotImplementedError","type object unavailable for "+name);});
put("NoneType",[](const Args& a,const Kw&){arity(a,0,0,"NoneType");return V();});
put("vars",[](const Args& a,const Kw&){arity(a,1,1,"vars");return attr(a[0],"__dict__");});
put("float",[](const Args& a,const Kw&){arity(a,0,1,"float");if(a.empty())return V(0.);if(a[0].kind()==Kind::String){try{size_t used=0;double d=std::stod(a[0].as_string(),&used);if(used!=a[0].as_string().size())fail("ValueError","invalid float string");return V(d);}catch(const std::invalid_argument&){fail("ValueError","invalid float string");}catch(const std::out_of_range&){fail("OverflowError","float string out of range");}}return V(a[0].as_double());});
put("int",[](const Args& a,const Kw&){arity(a,0,2,"int");if(a.empty())return V(0);if(a[0].kind()==Kind::String){try{size_t used=0;int64_t x=std::stoll(a[0].as_string(),&used,a.size()>1?int(a[1].as_int()):10);if(used!=a[0].as_string().size())fail("ValueError","invalid integer string");return V(x);}catch(const std::invalid_argument&){fail("ValueError","invalid integer string");}catch(const std::out_of_range&){fail("OverflowError","integer outside native signed64 range");}}if(integral(a[0]))return V(a[0].as_int());double d=a[0].as_double();if(!std::isfinite(d)||d<double(std::numeric_limits<int64_t>::min())||d>=9223372036854775808.0)fail("OverflowError","integer outside native signed64 range");return V(int64_t(d));});
put("str",[](const Args& a,const Kw&){arity(a,0,1,"str");return V(a.empty()?"":str(a[0]));});put("repr",[](const Args& a,const Kw&){arity(a,1,1,"repr");return V(repr(a[0]));});
put("list",[](const Args& a,const Kw&){arity(a,0,1,"list");return make_list(a.empty()?Args{}:iter(a[0]));});put("tuple",[](const Args& a,const Kw&){arity(a,0,1,"tuple");return make_tuple(a.empty()?Args{}:iter(a[0]));});put("set",[](const Args& a,const Kw&){arity(a,0,1,"set");return make_set(a.empty()?Args{}:iter(a[0]));});
put("dict",[](const Args& a,const Kw& k){arity(a,0,1,"dict");V d=make_dict();if(!a.empty())dict_merge(d,a[0]);for(auto& x:k)dict_set(d,x.first,x.second);return d;});
put("bytes",[](const Args& a,const Kw&){arity(a,0,2,"bytes");if(a.empty())return make_bytes("");if(a[0].kind()==Kind::String||a[0].kind()==Kind::Bytes)return make_bytes(a[0].as_string());if(integral(a[0]))return make_bytes(std::string(size_t(a[0].as_int()),'\0'));std::string s;for(auto& x:iter(a[0])){int64_t c=x.as_int();if(c<0||c>255)fail("ValueError","byte outside 0..255");s+=char(c);}return make_bytes(s);});
put("bytearray",[](const Args& a,const Kw& k){V v=call(get(builtins(),"bytes"),a,k);v.node()->kind=Kind::ByteArray;return v;});
put("range",[](const Args& a,const Kw&){arity(a,1,3,"range");int64_t first=a.size()==1?0:a[0].as_int(),stop=a.size()==1?a[0].as_int():a[1].as_int(),step=a.size()==3?a[2].as_int():1;if(!step)fail("ValueError","range step cannot be zero");Args out;for(int64_t i=first;step>0?i<stop:i>stop;i+=step)out.emplace_back(i);return make_list(std::move(out));});
put("abs",[](const Args& a,const Kw&){arity(a,1,1,"abs");return integral(a[0])?V(a[0].as_int()<0?int64_t(0-uint64_t(a[0].as_int())):a[0].as_int()):V(std::fabs(a[0].as_double()));});
put("sum",[](const Args& a,const Kw& k){arity(a,1,2,"sum");V s=a.size()>1?a[1]:kwget(k,"start",0);for(auto& x:iter(a[0]))s=binary("Add",s,x);return s;});
for(std::string name:{"min","max"})put(name,[name](const Args& a,const Kw& k){arity(a,1,size_t(-1),name);Args vals=a.size()==1?iter(a[0]):a;if(vals.empty()){for(auto& p:k)if(p.first=="default")return p.second;fail("ValueError",name+"() arg is an empty sequence");}V key=kwget(k,"key"),best=vals[0],bestkey=key.is_none()?best:call(key,{best});for(size_t i=1;i<vals.size();++i){V current=key.is_none()?vals[i]:call(key,{vals[i]});if(compare(name=="min"?"Lt":"Gt",current,bestkey)){best=vals[i];bestkey=current;}}return best;});
for(std::string name:{"any","all"})put(name,[name](const Args& a,const Kw&){arity(a,1,1,name);bool all=name=="all";if(a[0].kind()==Kind::Generator){V value;while(try_next(a[0],value)){bool x=truth(value);if(x!=all)return V(x);}}else for(auto& x:iter(a[0]))if(truth(x)!=all)return V(truth(x));return V(all);});
put("enumerate",[](const Args& a,const Kw& k){arity(a,1,2,"enumerate");auto n=std::make_shared<int64_t>((a.size()>1?a[1]:kwget(k,"start",0)).as_int());V source=iterator(a[0]);return make_generator([source,n](V& out){V value;if(!try_next(source,value))return false;out=make_tuple({V((*n)++),value});return true;});});
put("zip",[](const Args& a,const Kw& k){Args sources;for(auto& x:a)sources.push_back(iterator(x));bool strict=truth(kwget(k,"strict",false));return make_generator([sources,strict](V& out){if(sources.empty())return false;Args row;row.reserve(sources.size());for(size_t i=0;i<sources.size();++i){V value;if(try_next(sources[i],value)){row.push_back(std::move(value));continue;}if(strict){if(i)fail("ValueError","zip arguments differ in length");for(size_t j=1;j<sources.size();++j)if(try_next(sources[j],value))fail("ValueError","zip arguments differ in length");}return false;}out=make_tuple(std::move(row));return true;});});
put("map",[](const Args& a,const Kw&){arity(a,2,size_t(-1),"map");V fn=a[0];Args sources;for(size_t i=1;i<a.size();++i)sources.push_back(iterator(a[i]));return make_generator([fn,sources](V& out){Args row;row.reserve(sources.size());for(auto& source:sources){V value;if(!try_next(source,value))return false;row.push_back(std::move(value));}try{out=call(fn,row);return true;}catch(const PyError& e){if(e.kind=="StopIteration")return false;throw;}});});
put("filter",[](const Args& a,const Kw&){arity(a,2,2,"filter");V fn=a[0],source=iterator(a[1]);return make_generator([fn,source](V& out){V value;while(try_next(source,value)){try{if(truth(fn.is_none()?value:call(fn,{value}))){out=std::move(value);return true;}}catch(const PyError& e){if(e.kind=="StopIteration")return false;throw;}}return false;});});
put("sorted",[](const Args& a,const Kw& k){arity(a,1,1,"sorted");return make_list(sorted_values(a[0],kwget(k,"key"),kwget(k,"reverse",false)));});put("reversed",[](const Args& a,const Kw&){arity(a,1,1,"reversed");Args x=iter(a[0]);std::reverse(x.begin(),x.end());return make_list(std::move(x));});
put("round",[](const Args& a,const Kw&){arity(a,1,2,"round");double x=a[0].as_double();if(a.size()==1)return V(int64_t(std::nearbyint(x)));int64_t n=a[1].as_int();if(n>308)return a[0];if(n<-308)return V(std::copysign(0.,x));double scale=reference_pow(10.,double(n));double out=std::nearbyint(x*scale)/scale;return integral(a[0])?V(int64_t(out)):V(out);});
put("pow",[](const Args& a,const Kw&){arity(a,2,3,"pow");V out=binary("Pow",a[0],a[1]);return a.size()==3?binary("Mod",out,a[2]):out;});
put("isinstance",[](const Args& a,const Kw&){arity(a,2,2,"isinstance");std::function<bool(V,V)> check=[&](V v,V c){if(c.kind()==Kind::Tuple){for(auto& x:iter(c))if(check(v,x))return true;return false;}if(c.kind()==Kind::Class)return v.kind()==Kind::Instance&&subclass(v.node()->cls,c);if(c.kind()==Kind::Function){std::string n=c.node()->text;if(n=="int")return integral(v);if(n=="float")return v.kind()==Kind::Float;if(n=="bool")return v.kind()==Kind::Bool;return type_name(v)==n;}fail("TypeError","invalid isinstance type");};return V(check(a[0],a[1]));});
put("getattr",[](const Args& a,const Kw&){arity(a,2,3,"getattr");try{return attr(a[0],a[1].as_string());}catch(const PyError& e){if(e.kind=="AttributeError"&&a.size()>2)return a[2];throw;}});
put("setattr",[](const Args& a,const Kw&){arity(a,3,3,"setattr");setattr(a[0],a[1].as_string(),a[2]);return V();});put("hasattr",[](const Args& a,const Kw&){arity(a,2,2,"hasattr");try{attr(a[0],a[1].as_string());return V(true);}catch(const PyError& e){if(e.kind=="AttributeError")return V(false);throw;}});
put("callable",[](const Args& a,const Kw&){arity(a,1,1,"callable");return V(a[0].kind()==Kind::Function||a[0].kind()==Kind::Class||(a[0].kind()==Kind::Instance&&!method(a[0],"__call__").is_none()));});
put("next",[](const Args& a,const Kw&){arity(a,1,2,"next");V out;if(try_next(a[0],out))return out;if(a.size()>1)return a[1];fail("StopIteration","");});
put("iter",[](const Args& a,const Kw&){arity(a,1,1,"iter");return iterator(a[0]);});
put("classmethod",[](const Args& a,const Kw&){arity(a,1,1,"classmethod");return classmethod(a[0]);});put("staticmethod",[](const Args& a,const Kw&){arity(a,1,1,"staticmethod");return a[0];});
put("hex",[](const Args& a,const Kw&){arity(a,1,1,"hex");int64_t n=a[0].as_int();std::ostringstream s;if(n<0)s<<"-0x"<<std::hex<<uint64_t(0-uint64_t(n));else s<<"0x"<<std::hex<<n;return V(s.str());});
put("format",[](const Args& a,const Kw&){arity(a,1,2,"format");if(a.size()==1||a[1].as_string().empty())return V(str(a[0]));auto spec=a[1].as_string();std::ostringstream s;char type=spec.back();size_t dot=spec.find('.');if(dot!=std::string::npos)s<<std::setprecision(std::stoi(spec.substr(dot+1)));if(type=='f')s<<std::fixed<<a[0].as_double();else if(type=='e')s<<std::scientific<<a[0].as_double();else if(type=='g')s<<std::defaultfloat<<a[0].as_double();else if(type=='x'||type=='X'){if(type=='X')s<<std::uppercase;s<<std::hex<<a[0].as_int();}else if(type=='d')s<<a[0].as_int();else fail("NotImplementedError","unsupported format specifier "+spec);return V(s.str());});
for(std::string name:{"BaseException","Exception","ValueError","TypeError","KeyError","IndexError","ZeroDivisionError","OverflowError","ArithmeticError","LookupError","RuntimeError","AssertionError","AttributeError","StopIteration","ImportError","NameError","OSError","FileNotFoundError","NotImplementedError","InterruptedError","SystemExit"})put(name,[name](const Args& a,const Kw&){V e=node(Kind::Exception);e.node()->text=name;e.node()->attrs=child();set(e.node()->attrs,"message",a.empty()?V(""):V(str(a[0])));set(e.node()->attrs,"args",make_tuple(a));return e;});
V object=make_class("object",{},child());set(b,"object",object);set(b,"None",V());set(b,"True",true);set(b,"False",false);return b;}

namespace {
class JsonReader {const std::string& s;size_t p=0;void ws(){while(p<s.size()&&(s[p]==' '||s[p]=='\n'||s[p]=='\r'||s[p]=='\t'))++p;}[[noreturn]]void bad(){fail("ValueError","invalid JSON at byte "+std::to_string(p));}
unsigned hex4(){if(p+4>s.size())bad();unsigned n=0;for(int i=0;i<4;++i){char c=s[p++];n<<=4;if(c>='0'&&c<='9')n+=c-'0';else if(c>='a'&&c<='f')n+=c-'a'+10;else if(c>='A'&&c<='F')n+=c-'A'+10;else bad();}return n;}
static void utf8(std::string& out,unsigned n){if(n<0x80)out+=char(n);else if(n<0x800){out+=char(0xc0|(n>>6));out+=char(0x80|(n&63));}else if(n<0x10000){out+=char(0xe0|(n>>12));out+=char(0x80|((n>>6)&63));out+=char(0x80|(n&63));}else{out+=char(0xf0|(n>>18));out+=char(0x80|((n>>12)&63));out+=char(0x80|((n>>6)&63));out+=char(0x80|(n&63));}}
std::string string(){if(p>=s.size()||s[p++]!='"')bad();std::string out;while(p<s.size()){unsigned char c=s[p++];if(c=='"')return out;if(c<32)bad();if(c!='\\'){out+=c;continue;}if(p==s.size())bad();switch(s[p++]){case '"':out+='"';break;case '\\':out+='\\';break;case '/':out+='/';break;case 'b':out+='\b';break;case 'f':out+='\f';break;case 'n':out+='\n';break;case 'r':out+='\r';break;case 't':out+='\t';break;case 'u':{unsigned n=hex4();if(n>=0xd800&&n<=0xdbff&&p+2<=s.size()&&s.compare(p,2,"\\u")==0){p+=2;unsigned low=hex4();if(low<0xdc00||low>0xdfff)bad();n=0x10000+((n-0xd800)<<10)+(low-0xdc00);}utf8(out,n);break;}default:bad();}}bad();}
V value(){ws();if(p>=s.size())bad();char c=s[p];if(c=='"')return string();if(c=='['){++p;Args a;ws();if(p<s.size()&&s[p]==']'){++p;return make_list(a);}while(true){a.push_back(value());ws();if(p==s.size())bad();c=s[p++];if(c==']')break;if(c!=',')bad();}return make_list(std::move(a));}if(c=='{'){++p;V d=make_dict();ws();if(p<s.size()&&s[p]=='}'){++p;return d;}while(true){ws();std::string key=string();ws();if(p==s.size()||s[p++]!=':')bad();dict_set(d,key,value());ws();if(p==s.size())bad();c=s[p++];if(c=='}')break;if(c!=',')bad();}return d;}
for(auto x:std::vector<std::pair<std::string,V>>{{"null",V()},{"true",true},{"false",false},{"NaN",std::numeric_limits<double>::quiet_NaN()},{"Infinity",std::numeric_limits<double>::infinity()},{"-Infinity",-std::numeric_limits<double>::infinity()}})if(s.compare(p,x.first.size(),x.first)==0){p+=x.first.size();return x.second;}
size_t start=p;if(s[p]=='-')++p;if(p>=s.size()||!std::isdigit((unsigned char)s[p]))bad();if(s[p]=='0')++p;else while(p<s.size()&&std::isdigit((unsigned char)s[p]))++p;bool floating=false;if(p<s.size()&&s[p]=='.'){floating=true;++p;if(p==s.size()||!std::isdigit((unsigned char)s[p]))bad();while(p<s.size()&&std::isdigit((unsigned char)s[p]))++p;}if(p<s.size()&&(s[p]=='e'||s[p]=='E')){floating=true;++p;if(p<s.size()&&(s[p]=='+'||s[p]=='-'))++p;if(p==s.size()||!std::isdigit((unsigned char)s[p]))bad();while(p<s.size()&&std::isdigit((unsigned char)s[p]))++p;}
auto token=s.substr(start,p-start);try{if(!floating)return V(int64_t(std::stoll(token)));return V(std::stod(token));}catch(...){bad();}}
public:explicit JsonReader(const std::string& x):s(x){}V parse(){V x=value();ws();if(p!=s.size())bad();return x;}};
void dump_json(V v,std::string& out,bool sort,bool nan,std::unordered_set<const Node*>& stack){switch(v.kind()){case Kind::None:out+="null";return;case Kind::Bool:out+=truth(v)?"true":"false";return;case Kind::Int:out+=std::to_string(v.as_int());return;case Kind::Float:{double d=v.as_double();if(!std::isfinite(d)){if(!nan)fail("ValueError","non-finite value cannot be serialized");out+=std::isnan(d)?"NaN":d<0?"-Infinity":"Infinity";}else out+=floats(d,true);return;}case Kind::String:out+=escape(v.as_string(),'"');return;case Kind::List:case Kind::Tuple:case Kind::Dict:break;default:fail("TypeError","object of type "+type_name(v)+" is not JSON serializable");}
if(!stack.insert(v.node().get()).second)fail("ValueError","circular reference in JSON");if(sequence(v)){out+='[';bool first=true;for(auto& x:v.node()->seq){if(!first)out+=',';first=false;dump_json(x,out,sort,nan,stack);}out+=']';}else{out+='{';Args kk=keys(v);if(sort)std::stable_sort(kk.begin(),kk.end(),[](V a,V b){return compare("Lt",a,b);});bool first=true;for(auto& key:kk){if(!first)out+=',';first=false;std::string k;if(key.kind()==Kind::String)k=key.as_string();else if(key.is_none())k="null";else if(key.kind()==Kind::Bool)k=truth(key)?"true":"false";else if(numeric(key))k=str(key);else fail("TypeError","invalid JSON key");out+=escape(k,'"');out+=':';dump_json(item(v,key),out,sort,nan,stack);}out+='}';}stack.erase(v.node().get());}
}
V json_parse(const std::string& s){return JsonReader(s).parse();}
std::string json_dump(V v,bool sort_keys,bool allow_nan){std::string out;std::unordered_set<const Node*> stack;dump_json(v,out,sort_keys,allow_nan,stack);return out;}
}
