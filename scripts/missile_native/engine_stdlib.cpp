#include "engine_stdlib.h"
#include "reference_norm.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <regex>
#include <sstream>
#include <thread>

namespace nt {
double reference_pow(double x,double y);
thread_local std::function<V(V)> envelope_yield;
static std::string engine_root;
std::string engine_source_path(const std::string& relative){return (std::filesystem::u8path(engine_root)/std::filesystem::u8path(relative)).string();}
static V arg(const Args& a,size_t i,V fallback={}) { return i<a.size()?a[i]:fallback; }
static V kw(const Kw& k,const std::string& name,V fallback={}) { for(auto& p:k)if(p.first==name)return p.second;return fallback; }
static void put(E e,const char* name,Builtin fn){set(e,name,builtin(name,std::move(fn)));}
static V object(const std::string& name,Kw values){auto e=child();for(auto& p:values)set(e,p.first,p.second);return make_module(name,e);}
static std::string raw(V v){if(v.kind()==Kind::String)return v.as_string();if(v.kind()==Kind::Bytes)return v.node()->text;std::string s;for(V x:iter(v))s.push_back(char(x.as_int()));return s;}
static double norm(std::vector<double> x){double maximum=0.;int nan=0;for(double& v:x){v=std::fabs(v);if(v>maximum)maximum=v;nan|=std::isnan(v);}return vector_norm(x.size(),x.data(),maximum,nan);}

// SHA-256, FIPS 180-4. Input and digest bytes remain native throughout.
static std::string sha256(const std::string& input){
 static const uint32_t k[]={0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2};
 uint32_t h[]={0x6a09e667,0xbb67ae85,0x3c6ef372,0xa54ff53a,0x510e527f,0x9b05688c,0x1f83d9ab,0x5be0cd19};
 auto ro=[](uint32_t x,unsigned n){return(x>>n)|(x<<(32-n));};
 std::string s=input;uint64_t bits=uint64_t(s.size())*8;s.push_back(char(128));while(s.size()%64!=56)s.push_back(0);for(int i=7;i>=0;--i)s.push_back(char(bits>>(i*8)));
 for(size_t off=0;off<s.size();off+=64){uint32_t w[64];for(int i=0;i<16;++i){w[i]=0;for(int j=0;j<4;++j)w[i]=(w[i]<<8)|uint8_t(s[off+4*i+j]);}for(int i=16;i<64;++i)w[i]=w[i-16]+(ro(w[i-15],7)^ro(w[i-15],18)^(w[i-15]>>3))+w[i-7]+(ro(w[i-2],17)^ro(w[i-2],19)^(w[i-2]>>10));uint32_t a=h[0],b=h[1],c=h[2],d=h[3],e=h[4],f=h[5],g=h[6],z=h[7];for(int i=0;i<64;++i){uint32_t t=z+(ro(e,6)^ro(e,11)^ro(e,25))+((e&f)^(~e&g))+k[i]+w[i],u=(ro(a,2)^ro(a,13)^ro(a,22))+((a&b)^(a&c)^(b&c));z=g;g=f;f=e;e=d+t;d=c;c=b;b=a;a=t+u;}uint32_t v[]={a,b,c,d,e,f,g,z};for(int i=0;i<8;++i)h[i]+=v[i];}
 std::ostringstream out;for(auto v:h)out<<std::hex<<std::setw(8)<<std::setfill('0')<<v;return out.str();
}

static V path_value(const std::filesystem::path& path);
static thread_local V path_class;
static V path_value(const std::filesystem::path& path){auto n=std::make_shared<Node>(Kind::Instance);n->cls=path_class;n->attrs=child();set(n->attrs,"_path",path.generic_string());return V(n);}
static std::filesystem::path path_of(V v){return std::filesystem::u8path(attr(v,"_path").as_string());}
static std::string read_file(const std::filesystem::path& path){std::ifstream f(path,std::ios::binary);if(!f)throw PyError("OSError","Cannot read "+path.string());return std::string(std::istreambuf_iterator<char>(f),{});}

static std::vector<char> format(const std::string& f){std::vector<char> out;size_t i=0;if(!f.empty()&&(f[0]=='<'||f[0]=='='||f[0]=='@'))++i;else if(!f.empty()&&(f[0]=='>'||f[0]=='!'))throw PyError("NotImplementedError","Big-endian struct format");while(i<f.size()){size_t count=0;while(i<f.size()&&isdigit((unsigned char)f[i]))count=count*10+f[i++]-'0';if(i==f.size())throw PyError("ValueError","Invalid struct format");char c=f[i++];for(size_t j=0;j<(count?count:1);++j)out.push_back(c);}return out;}
static size_t width(char c){switch(c){case 'f':case 'i':case 'I':case 'l':case 'L':return 4;case 'd':case 'q':case 'Q':return 8;case 'h':case 'H':return 2;case 'b':case 'B':case '?':case 'x':return 1;default:throw PyError("NotImplementedError",std::string("struct format ")+c);}}
static std::string pack(const std::string& fmt,const Args& a,size_t start){std::string s;for(char c:format(fmt)){size_t w=width(c);uint64_t u=0;if(c!='x'){if(start>=a.size())throw PyError("ValueError","Too few struct items");V v=a[start++];if(c=='f'){float x=(float)v.as_double();if(!std::isfinite(x)&&std::isfinite(v.as_double()))throw PyError("OverflowError","float too large to pack");uint32_t b;std::memcpy(&b,&x,4);u=b;}else if(c=='d'){double x=v.as_double();std::memcpy(&u,&x,8);}else u=(uint64_t)v.as_int();}for(size_t j=0;j<w;++j)s.push_back(char(u>>(j*8)));}if(start!=a.size())throw PyError("ValueError","Too many struct items");return s;}
static V unpack(const std::string& fmt,const std::string& s,size_t off,bool exact){Args out;for(char c:format(fmt)){size_t w=width(c);if(off+w>s.size())throw PyError("ValueError","Short struct buffer");uint64_t u=0;for(size_t j=0;j<w;++j)u|=uint64_t(uint8_t(s[off+j]))<<(j*8);off+=w;if(c=='x')continue;if(c=='f'){float x;uint32_t b=(uint32_t)u;std::memcpy(&x,&b,4);out.emplace_back(double(x));}else if(c=='d'){double x;std::memcpy(&x,&u,8);out.emplace_back(x);}else if(c=='?')out.emplace_back(bool(u));else{if(c=='i'||c=='l')out.emplace_back(int64_t(int32_t(u)));else if(c=='h')out.emplace_back(int64_t(int16_t(u)));else if(c=='b')out.emplace_back(int64_t(int8_t(u)));else out.emplace_back(int64_t(u));}}if(exact&&off!=s.size())throw PyError("ValueError","Incorrect struct buffer size");return make_tuple(std::move(out));}

void register_native_stdlib(const std::string& root){
 engine_root=root;
 register_module("math",[](E e){
  set(e,"pi",3.141592653589793238462643383279502884);set(e,"inf",std::numeric_limits<double>::infinity());
  auto unary=[e](const char* name,double(*f)(double)){bool zero_domain=std::string(name)=="log";put(e,name,[f,zero_domain](const Args&a,const Kw&){double x=arg(a,0).as_double();if(zero_domain&&x==0.)throw PyError("ValueError","math domain error");double r=f(x);if(std::isnan(r)&&!std::isnan(x))throw PyError("ValueError","math domain error");if(std::isinf(r)&&std::isfinite(x))throw PyError("OverflowError","math range error");return V(r);});};
  unary("sqrt",std::sqrt);unary("sin",std::sin);unary("cos",std::cos);unary("acos",std::acos);unary("asin",std::asin);unary("exp",std::exp);unary("log",std::log);
  put(e,"isfinite",[](const Args&a,const Kw&){return V(std::isfinite(arg(a,0).as_double()));});
  put(e,"degrees",[](const Args&a,const Kw&){return V(arg(a,0).as_double()*(180.0/3.141592653589793238462643383279502884));});
  put(e,"radians",[](const Args&a,const Kw&){return V(arg(a,0).as_double()*(3.141592653589793238462643383279502884/180.0));});
  for(auto name:{"ceil","floor","trunc"})put(e,name,[name](const Args&a,const Kw&){double x=arg(a,0).as_double();if(!std::isfinite(x))throw PyError("ValueError","Nonfinite integer conversion");return V(int64_t(std::string(name)=="ceil"?std::ceil(x):std::string(name)=="floor"?std::floor(x):std::trunc(x)));});
  put(e,"frexp",[](const Args&a,const Kw&){int exponent;double x=std::frexp(arg(a,0).as_double(),&exponent);return make_tuple({x,exponent});});
  put(e,"copysign",[](const Args&a,const Kw&){return V(std::copysign(arg(a,0).as_double(),arg(a,1).as_double()));});
  put(e,"atan2",[](const Args&a,const Kw&){return V(std::atan2(arg(a,0).as_double(),arg(a,1).as_double()));});
  put(e,"fmod",[](const Args&a,const Kw&){double r=std::fmod(arg(a,0).as_double(),arg(a,1).as_double());if(std::isnan(r))throw PyError("ValueError","math domain error");return V(r);});
  put(e,"pow",[](const Args&a,const Kw&){double r=reference_pow(arg(a,0).as_double(),arg(a,1).as_double());if(std::isnan(r))throw PyError("ValueError","math domain error");return V(r);});
  put(e,"hypot",[](const Args&a,const Kw&){std::vector<double> x;for(V v:a)x.push_back(v.as_double());return V(norm(x));});
  put(e,"dist",[](const Args&a,const Kw&){Args x=iter(arg(a,0)),y=iter(arg(a,1));if(x.size()!=y.size())throw PyError("ValueError","Both points must have same dimension");std::vector<double>d;for(size_t i=0;i<x.size();++i)d.push_back(x[i].as_double()-y[i].as_double());return V(norm(d));});
 });
 register_module("struct",[](E e){put(e,"pack",[](const Args&a,const Kw&){return make_bytes(pack(arg(a,0).as_string(),a,1));});put(e,"unpack",[](const Args&a,const Kw&){return unpack(arg(a,0).as_string(),raw(arg(a,1)),0,true);});put(e,"unpack_from",[](const Args&a,const Kw&){return unpack(arg(a,0).as_string(),raw(arg(a,1)),size_t(arg(a,2,0).as_int()),false);});put(e,"pack_into",[](const Args&a,const Kw&){Args values(a.begin()+3,a.end());std::string s=pack(arg(a,0).as_string(),values,0);int64_t off=arg(a,2).as_int();for(size_t i=0;i<s.size();++i)setitem(arg(a,1),V(off+int64_t(i)),int(uint8_t(s[i])));return V();});});
 register_module("json",[](E e){put(e,"loads",[](const Args&a,const Kw&){return json_parse(raw(arg(a,0)));});put(e,"dumps",[](const Args&a,const Kw&k){return V(json_dump(arg(a,0),truth(kw(k,"sort_keys",false)),truth(kw(k,"allow_nan",true))));});});
 register_module("hashlib",[](E e){put(e,"sha256",[](const Args&a,const Kw&){std::string hex=sha256(raw(arg(a,0,make_bytes(""))));return object("sha256",{{"hexdigest",builtin("hexdigest",[hex](const Args&,const Kw&){return V(hex);})}});});});
 register_module("copy",[](E e){put(e,"deepcopy",[](const Args&a,const Kw&){return deepcopy(arg(a,0));});put(e,"copy",[](const Args&a,const Kw&){V v=arg(a,0);if(v.kind()==Kind::Instance){auto n=std::make_shared<Node>(*v.node());n->attrs=child(v.node()->attrs->parent);n->attrs->values=v.node()->attrs->values;return V(n);}if(v.kind()==Kind::List)return make_list(iter(v));if(v.kind()==Kind::Dict){V out=make_dict();dict_merge(out,v);return out;}return v;});});
 register_module("functools",[](E e){put(e,"lru_cache",[](const Args&,const Kw&){return builtin("cache_decorator",[](const Args&a,const Kw&){V fn=arg(a,0);auto values=std::make_shared<std::unordered_map<std::string,V>>();return builtin("cached",[fn,values](const Args&a,const Kw&k){std::string key=json_dump(make_tuple(a))+json_dump(make_dict(k));auto it=values->find(key);if(it!=values->end())return it->second;V v=call(fn,a,k);(*values)[key]=v;return v;});});});});
 register_module("dataclasses",[](E e){put(e,"asdict",[](const Args&a,const Kw&){std::function<V(V)> visit=[&](V v)->V{if(v.kind()==Kind::Instance){V out=make_dict();for(auto& p:v.node()->attrs->values)if(p.first.rfind("__",0)!=0)dict_set(out,p.first,visit(p.second));return out;}if(v.kind()==Kind::Tuple||v.kind()==Kind::List){Args rows;for(V x:iter(v))rows.push_back(visit(x));return v.kind()==Kind::Tuple?make_tuple(rows):make_list(rows);}if(v.kind()==Kind::Dict){V out=make_dict();for(auto& p:v.node()->entries)dict_set(out,p.first,visit(p.second));return out;}return deepcopy(v);};return visit(arg(a,0));});put(e,"dataclass",[](const Args&a,const Kw&){return a.empty()?builtin("dataclass_decorator",[](const Args&a,const Kw&){return arg(a,0);}):a[0];});});
 register_module("pathlib",[](E e){auto c=child();path_class=make_class("Path",{},c);
  set(c,"__str__",builtin("Path.__str__",[](const Args&a,const Kw&){return attr(arg(a,0),"_path");}));
  set(c,"__truediv__",builtin("Path.__truediv__",[](const Args&a,const Kw&){return path_value(path_of(arg(a,0))/std::filesystem::u8path(str(arg(a,1))));}));
  set(c,"__getattr__",builtin("Path.__getattr__",[](const Args&a,const Kw&){auto p=path_of(arg(a,0));std::string name=arg(a,1).as_string();if(name=="name")return V(p.filename().generic_string());if(name=="parent")return path_value(p.parent_path());if(name=="stem")return V(p.stem().generic_string());throw PyError("AttributeError","Path."+name);}));
  set(c,"read_text",builtin("Path.read_text",[](const Args&a,const Kw&){return V(read_file(path_of(arg(a,0))));}));set(c,"read_bytes",builtin("Path.read_bytes",[](const Args&a,const Kw&){return make_bytes(read_file(path_of(arg(a,0))));}));
  set(c,"resolve",builtin("Path.resolve",[](const Args&a,const Kw&){return path_value(std::filesystem::absolute(path_of(arg(a,0))).lexically_normal());}));
  set(c,"with_name",builtin("Path.with_name",[](const Args&a,const Kw&){return path_value(path_of(arg(a,0)).parent_path()/std::filesystem::u8path(arg(a,1).as_string()));}));
  set(c,"is_file",builtin("Path.is_file",[](const Args&a,const Kw&){return V(std::filesystem::is_regular_file(path_of(arg(a,0))));}));
  set(e,"Path",builtin("Path",[](const Args&a,const Kw&){return path_value(std::filesystem::u8path(a.empty()?".":str(a[0])));}));
 });
 register_module("re",[](E e){put(e,"fullmatch",[](const Args&a,const Kw&){return std::regex_match(arg(a,1).as_string(),std::regex(arg(a,0).as_string()))?V(true):V();});});
 register_module("time",[](E e){for(auto name:{"perf_counter","monotonic","time"})put(e,name,[](const Args&,const Kw&){return V(std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count());});});
 register_module("os",[](E e){set(e,"name",
#ifdef _WIN32
 "nt"
#else
 "posix"
#endif
 );V env=make_dict();for(auto key:{"WT_ENVELOPE_WORKERS","WT_MISSILE_ENGINE"}){auto value=std::getenv(key);if(value)dict_set(env,key,value);}set(e,"environ",env);put(e,"cpu_count",[](const Args&,const Kw&){return V(int(std::thread::hardware_concurrency()));});});
 register_module("sys",[](E e){set(e,"path",make_list());set(e,"modules",make_dict());});
 register_module("argparse",[](E){});
 register_module("missile_inputs",[](E e){put(e,"directory",[](const Args&,const Kw&){import_module("pathlib");auto p=std::filesystem::u8path(engine_root)/"references/missile-inputs";if(!std::filesystem::is_regular_file(p/"launch-profiles/index.json"))p=std::filesystem::u8path(engine_root)/"scripts/missile_model";else if(!std::filesystem::is_regular_file(p/"radar-launch-properties.json"))throw PyError("ValueError","Managed missile inputs are incomplete");return path_value(p);});});
 set(builtins(),"__yield__",builtin("__yield__",[](const Args&a,const Kw&){if(!envelope_yield)throw PyError("RuntimeError","Envelope yield outside native search");return envelope_yield(arg(a,0));}));
}
}
