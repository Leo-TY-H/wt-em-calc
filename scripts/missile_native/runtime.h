#pragma once
// Standalone native semantics for the project's translated simulation sources.
// No CPython headers, Python runtime, or Python object representations are used.
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

namespace nt {
struct Node;
struct Env;
struct V;
using E = std::shared_ptr<Env>;
using Args = std::vector<V>;
using Kw = std::vector<std::pair<std::string,V>>;
enum class Kind { None, Bool, Int, Float, String, Bytes, ByteArray, List, Tuple, Dict, Set,
                  Function, Class, Instance, Module, Generator, Super, Exception };
struct V {
    using Data = std::variant<std::monostate,bool,int64_t,double,std::string,std::shared_ptr<Node>>;
    Data data;
    V() = default;
    V(std::nullptr_t) {}
    V(bool x):data(x){}
    V(int x):data(int64_t(x)){}
    V(unsigned x):data(int64_t(x)){}
    V(int64_t x):data(x){}
    V(double x):data(x){}
    V(float x):data(double(x)){}
    V(const char* x):data(std::string(x)){}
    V(std::string x):data(std::move(x)){}
    explicit V(std::shared_ptr<Node> x):data(std::move(x)){}
    Kind kind() const;
    bool is_none() const { return std::holds_alternative<std::monostate>(data); }
    double as_double() const;
    int64_t as_int() const;
    const std::string& as_string() const;
    const std::shared_ptr<Node>& node() const;
};
struct Env {
    std::unordered_map<std::string,V> values;
    E parent;
    bool function_frame = false;
    bool closure_tracked = false;
};
using Fn = std::function<V(E,const Args&,const Kw&)>;
using Builtin = std::function<V(const Args&,const Kw&)>;
struct Node {
    Kind kind;
    Args seq;
    std::vector<std::pair<V,V>> entries;
    std::unordered_map<std::string,size_t> string_index;
    std::string text;
    E attrs;
    E closure;
    Fn fn;
    Builtin builtin;
    V (*unary_fast)(const V&) = nullptr;
    V (*binary_fast)(const V&,const V&) = nullptr;
    std::function<bool(V&)> advance;
    V cls;
    Args bases;
    bool class_method = false;
    explicit Node(Kind k):kind(k){}
};
struct PyError : std::runtime_error {
    std::string kind;
    V value;
    PyError(std::string kind,std::string message);
};
E child(E parent = {});
E root_env(E);
void track_capture(E);
V get(const E&,const std::string&);
void set(const E&,const std::string&,V);
void set_outer(E,const std::string&,V);
void del(E,const std::string&);
void bind(E,const Args&,const Kw&,const std::vector<std::string>& positional,
          int positional_only,const std::vector<std::string>& keyword_only,
          const Kw& defaults,const std::string& vararg="",const std::string& kwarg="");
struct FrameGuard { E env; explicit FrameGuard(E e):env(std::move(e)){} ~FrameGuard(); };
// Public engine requests return data, never callbacks. Release captured frames
// at that boundary, preserving shared/nonlocal bindings during the calculation.
struct InvocationGuard {
    size_t start;
    InvocationGuard();
    ~InvocationGuard();
    InvocationGuard(const InvocationGuard&) = delete;
    InvocationGuard& operator=(const InvocationGuard&) = delete;
};
V builtin(const std::string&,Builtin);
V make_function(const std::string&,E,Fn);
V make_class(const std::string&,const Args&,E);
V make_module(const std::string&,E);
V make_list(Args = {});
V make_tuple(Args = {});
V make_set(Args = {});
V make_dict(Kw = {});
V make_bytes(std::string);
V make_generator(std::function<V()>);
V make_generator(std::function<bool(V&)>);
bool try_next(V,V&);
V next(V);
V iterator(V);
V call(V,const Args& = {},const Kw& = {});
[[noreturn]] void rethrow_call(const PyError&,const std::string&);
inline V call1(const V& function,const V& argument) {
    auto pointer=std::get_if<std::shared_ptr<Node>>(&function.data);
    if(pointer && (*pointer)->unary_fast) {
        try { return (*pointer)->unary_fast(argument); }
        catch(const PyError& error) { rethrow_call(error,(*pointer)->text); }
    }
    return call(function,Args{argument});
}
inline V call2(const V& function,const V& first,const V& second) {
    auto pointer=std::get_if<std::shared_ptr<Node>>(&function.data);
    if(pointer && (*pointer)->binary_fast) {
        try { return (*pointer)->binary_fast(first,second); }
        catch(const PyError& error) { rethrow_call(error,(*pointer)->text); }
    }
    return call(function,Args{first,second});
}
// unordered_map element references survive insertion/rehashing. Generated
// functions retain these pointers only for their own live invocation frame;
// closures keep using that same frame, including nonlocal assignments.
inline V read_local(const E& env,const char* name,V*& cached) {
    if(!cached) {
        auto found=env->values.find(name);
        if(found==env->values.end())return get(env,name);
        cached=&found->second;
    }
    return *cached;
}
inline void write_local(const E& env,const char* name,V*& cached,V value) {
    if(!cached)cached=&env->values[name];
    *cached=std::move(value);
}
V attr(V,const std::string&);
void setattr(V,const std::string&,V);
void delattr(V,const std::string&);
V item(V,V);
void setitem(V,V,V);
void delitem(V,V);
V slice(V,V start = {},V stop = {},V step = {});
void setslice(V,V start,V stop,V step,V values);
void dict_set(V,V,V);
void dict_merge(V,V);
V dict_get(V,V,V fallback = {});
bool contains(V,V);
bool truth(V);
bool equal(V,V);
bool identical(V,V);
V binary(const std::string&,V,V);
V inplace(const std::string&,V,V);
V unary(const std::string&,V);
bool compare(const std::string&,V,V);
Args iter(V);
std::string str(V);
std::string repr(V);
std::string type_name(V);
V deepcopy(V);
V classmethod(V);
V dataclass(V,const std::vector<std::string>& fields,const Kw& defaults,bool frozen=false);
V super_of(V cls,V self);
V exception_value(const PyError&);
bool exception_matches(const PyError&,V);
[[noreturn]] void raise(V);
using ModuleInit = std::function<void(E)>;
void register_module(const std::string&,ModuleInit);
V import_module(const std::string&);
E module_env(const std::string&);
E builtins();
V json_parse(const std::string&);
std::string json_dump(V,bool sort_keys=false,bool allow_nan=true);
}
