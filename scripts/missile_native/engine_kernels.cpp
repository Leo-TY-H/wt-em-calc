// Native adapters around the independently verified C numerical kernels.
// All arguments and results are native values; this file never uses CPython.
#include "engine_kernels.h"
#include "typed_engine.h"
#include <cmath>
#include <math.h>
namespace nt { double reference_pow(double,double); }
// Bind the shared C kernels to the same native host pow implementation used
// by the translated operations; the Cython/reference C build is unchanged.
#ifdef _WIN32
#define pow nt::reference_pow
#endif
#include "controls.h"
#ifdef _WIN32
#undef pow
#endif
#include <array>
#include <map>
#include <tuple>

namespace nt {
namespace {
V field(V value, const char* key) {
    return value.kind() == Kind::Dict ? item(value, V(key)) : attr(value, key);
}
float number(V value) { return static_cast<float>(value.as_double()); }
float num(V value, const char* key) { return number(item(value, V(key))); }
float at(V value, int index) { return number(item(value, V(index))); }
size_t count(V value) { return iter(value).size(); }
void check() {
    int code = wtm_error_code;
    const char* message = wtm_error_text;
    wtm_error_code = 0;
    wtm_error_text = nullptr;
    if (code) throw PyError(code == WTM_ZERO_DIVISION ? "ZeroDivisionError" :
                            code == WTM_UNSUPPORTED ? "NotImplementedError" : "ValueError",
                            message ? message : "Native numerical error");
}
v3 vec3(V value) { return V3(at(value,0),at(value,1),at(value,2)); }
q4 quat(V value) { return {at(value,0),at(value,1),at(value,2),at(value,3)}; }
V list3(v3 value) { return make_list({V(value.x),V(value.y),V(value.z)}); }
V list4(q4 value) { return make_list({V(value.x),V(value.y),V(value.z),V(value.w)}); }
template<size_t N> V floats(const float (&values)[N]) {
    Args result; result.reserve(N);
    for (float value : values) result.emplace_back(value);
    return make_list(std::move(result));
}
template<size_t N> void read_floats(V value, float (&values)[N]) {
    for (size_t i=0;i<N;++i) values[i]=at(value,static_cast<int>(i));
}
// Small kernel calls avoid allocating an environment for positional arguments.
Args arguments(const Args& args, const Kw& kw, std::initializer_list<const char*> names,
               const Kw& defaults = {}, size_t keyword_start = SIZE_MAX) {
    if (args.size()>names.size() || args.size()>keyword_start)
        throw PyError("TypeError","Too many positional arguments");
    Args bound(names.size()); std::vector<bool> present(names.size(),false);
    size_t i=0;
    for (const V& arg:args) { bound[i]=arg; present[i++]=true; }
    auto locate=[&](const std::string& key)->size_t {
        size_t n=0; for (const char* name:names) { if(key==name)return n; ++n; }
        throw PyError("TypeError","Unexpected keyword argument: "+key);
    };
    for (const auto& pair:kw) {
        size_t n=locate(pair.first);
        if(present[n])throw PyError("TypeError","Multiple values for argument: "+pair.first);
        bound[n]=pair.second; present[n]=true;
    }
    for(const auto& pair:defaults) {
        size_t n=locate(pair.first);
        if(!present[n]) {bound[n]=pair.second;present[n]=true;}
    }
    i=0; for(const char* name:names) {
        if(!present[i++])throw PyError("TypeError",std::string("Missing required argument: ")+name);
    }
    return bound;
}
void expose(E env,const char* name,Builtin function) {set(env,name,builtin(name,std::move(function)));}
V fast_f32(const V& value){return V(number(value));}
template<int operation> V fast_arithmetic(const V& a,const V& b){
    float y=number(b);
    if(operation==4 && !(fabsf(y)>F(4e-19)))return V(0.);
    float x=number(a),result;
    if(operation==0)result=fadd(x,y);else if(operation==1)result=fsub(x,y);
    else if(operation==2)result=fmul(x,y);else if(operation==3)result=fdiv(x,y);
    else result=::safe_div(x,y);
    check();return V(result);
}
// These visitors preserve the complete validation traversal. Most state words
// are finite; avoid constructing paths, closures, tuples and argument frames
// for each successful scalar check. Invalid inputs use the existing translated
// validator to retain its detailed error and failure classification.
enum class NumberPath { Other, Root, Guidance, Axes, Propulsion, PidPair, PidRow, Allowed };
NumberPath number_child(NumberPath parent,const V& key) {
    if(key.kind()==Kind::String) {
        const auto& name=key.as_string();
        if(parent==NumberPath::Root && name=="guidance")return NumberPath::Guidance;
        if(parent==NumberPath::Guidance) {
            if(name=="orientation" || name=="guidance")return NumberPath::Axes;
            if(name=="propulsion")return NumberPath::Propulsion;
        }
        if(name=="pid") {
            if(parent==NumberPath::Axes)return NumberPath::PidPair;
            if(parent==NumberPath::Propulsion)return NumberPath::PidRow;
        }
    } else if(key.kind()==Kind::Int) {
        int64_t index=key.as_int();
        if(parent==NumberPath::PidPair && index>=0 && index<2)return NumberPath::PidRow;
        if(parent==NumberPath::PidRow && index>=0 && index<8)return NumberPath::Allowed;
    }
    return NumberPath::Other;
}
bool valid_numbers(const V& value,NumberPath path,bool plain) {
    Kind kind=value.kind();
    if(kind==Kind::Float)return std::isfinite(value.as_double()) || (!plain && path==NumberPath::Allowed);
    if(kind==Kind::Dict) {
        const auto& node=value.node();
        if(node->attrs) {
            for(const auto& pair:node->attrs->values)
                if(!valid_numbers(pair.second,plain?NumberPath::Other:number_child(path,V(pair.first)),plain))return false;
        } else {
            for(const auto& pair:node->entries) {
                if(plain && pair.first.kind()!=Kind::String)return false;
                if(!valid_numbers(pair.second,plain?NumberPath::Other:number_child(path,pair.first),plain))return false;
            }
        }
    } else if(kind==Kind::List || kind==Kind::Tuple) {
        const auto& items=value.node()->seq;
        for(size_t i=0;i<items.size();++i)
            if(!valid_numbers(items[i],plain?NumberPath::Other:number_child(path,V(int64_t(i))),plain))return false;
    }
    return true;
}
void bound_invocation(E env,const char* class_name,const char* method_name) {
    V cls=get(env,class_name),original=attr(cls,method_name);
    // These session boundaries return numerical/state records only. Incoming
    // target/provider callbacks and the persistent radar-step closure exist
    // before this scope; only transient captures made during the completed
    // update are released. Prepared radar properties are plain native values
    // retained by the outer callback, so clearing a transient frame preserves
    // its immutable property cache and the committed session state.
    setattr(cls,method_name,builtin(std::string(class_name)+"."+method_name,
        [original](const Args& args,const Kw& kwargs) {
            InvocationGuard invocation;
            return call(original,args,kwargs);
        }));
}

void load_motors(V value,wtm_motors& output) {
    Args motors=iter(value);
    if(motors.size()>4)throw PyError("ValueError","At most four propulsion units");
    output={};output.count=static_cast<int>(motors.size());
    for(size_t i=0;i<motors.size();++i) {
        auto& p=output.items[i]; V m=motors[i]; Args impulses=iter(field(m,"impulses"));
        p.delay=number(field(m,"delay"));
        if(impulses.size()>4)throw PyError("ValueError","At most four impulses");
        p.count=static_cast<int>(impulses.size());
        for(size_t j=0;j<impulses.size();++j) {
            V impulse=impulses[j]; auto& out=p.impulses[j];
            if(field(impulse,"factor_index").as_int()>=0)
                throw PyError("NotImplementedError","Propulsion factor tables need their native loader");
            out.duration=number(field(impulse,"duration"));out.force=number(field(impulse,"force"));
            out.mass_lost=number(field(impulse,"mass_lost"));out.vectoring=number(field(impulse,"vectoring"));
        }
    }
}
void load_rocket(V value,wtm_rocket& output) {
    read_floats(dict_get(value,"extPressureToThrustMult",make_list({0.,1.,0.,1.})),output.ext_pressure);
    output.thrust_deviation=number(dict_get(value,"thrustDeviation",0.));
    output.orientation_torque=number(dict_get(value,"orientationTorque",0.));
}
void load_rows(V value,wtm_row* rows,int& length,int maximum,const char* error) {
    Args values=iter(value);if(values.size()>static_cast<size_t>(maximum))throw PyError("ValueError",error);
    length=static_cast<int>(values.size());
    for(size_t i=0;i<values.size();++i)rows[i]={at(values[i],0),at(values[i],1),at(values[i],2)};
}
void load_body_props(V value,wtm_body_props& output) {
#define SCALAR(name) output.name=num(value,#name)
    SCALAR(mass);SCALAR(front_area);SCALAR(side_area);SCALAR(stabilizer_arm);SCALAR(length);
    SCALAR(cx);SCALAR(cy);SCALAR(cy_limit);SCALAR(cx_aoa);SCALAR(fins_hor);SCALAR(fins_ver);
    SCALAR(fin_pressure_limit);SCALAR(damping_geometry);
#undef SCALAR
    read_floats(item(value,"inertia"),output.inertia);
    read_floats(item(value,"angular_damping"),output.angular_damping);
    output.axis_quaternion=quat(item(value,"axis_quaternion"));
    load_rows(item(value,"cy_table"),output.cy_table,output.cy_rows,64,"cy table too long");
}
void load_body(V value,wtm_body& output) {
    output={};output.position=vec3(item(value,"position"));output.quaternion=quat(item(value,"quaternion"));
    output.velocity=vec3(item(value,"velocity"));output.omega=vec3(item(value,"omega"));
    output.time=num(value,"time");output.seed=static_cast<uint32_t>(item(value,"seed").as_int());
    output.body_random=num(value,"body_random");read_floats(item(value,"clocks"),output.clocks);
    output.water=truth(item(value,"water"));output.distance=num(value,"distance");
    output.water_distance=num(value,"water_distance");output.surface_depth=num(value,"surface_depth");
    output.immersion=num(value,"immersion");
}
void store_body(V value,const wtm_body& b) {
    dict_set(value,"position",list3(b.position));dict_set(value,"quaternion",list4(b.quaternion));
    dict_set(value,"velocity",list3(b.velocity));dict_set(value,"omega",list3(b.omega));
    dict_set(value,"time",b.time);dict_set(value,"clocks",floats(b.clocks));
    dict_set(value,"distance",b.distance);dict_set(value,"immersion",b.immersion);
}
V packet(V b) {
    Args result;
    for(const char* key:{"position","quaternion","velocity","omega"}) {
        Args part=iter(item(b,key));result.insert(result.end(),part.begin(),part.end());
    }
    result.push_back(item(b,"time"));return make_list(std::move(result));
}
struct FlightProperties { V body_source,rocket_source,motor_source;wtm_body_props body;wtm_rocket rocket;wtm_motors motors; };
using FlightKey=std::tuple<Node*,Node*,Node*>;
thread_local std::map<FlightKey,FlightProperties> flight_cache;
template<typename T> struct Prepared {V source;T value;};
thread_local std::map<Node*,Prepared<wtm_motors>> motor_cache;
thread_local std::map<Node*,Prepared<wtm_orientation_props>> orientation_cache;
thread_local std::map<Node*,Prepared<wtm_guidance_props>> guidance_cache;
thread_local std::map<Node*,Prepared<wtm_acceleration_props>> acceleration_cache;
thread_local std::map<Node*,Prepared<wtm_aero_props>> aero_cache;
thread_local std::map<Node*,Prepared<wtm_propulsion_control_props>> propulsion_cache;
thread_local std::map<Node*,Prepared<wtm_body_props>> body_props_cache;
template<typename T,typename Loader>
const T& prepared(std::map<Node*,Prepared<T>>& cache,V source,Loader load) {
    Node* key=source.node().get();auto found=cache.find(key);
    if(found!=cache.end())return found->second.value;
    Prepared<T> entry{source,{}};load(source,entry.value);
    if(cache.size()>=64)cache.clear();
    return cache.emplace(key,std::move(entry)).first->second.value;
}
const FlightProperties& flight_properties(V p) {
    V body=item(p,"body"),rocket=item(p,"rocket"),motors=dict_get(p,"_motor_properties");
    if(motors.is_none())motors=item(item(p,"guidance"),"motors");
    FlightKey key{body.node().get(),rocket.node().get(),motors.node().get()};auto found=flight_cache.find(key);
    if(found!=flight_cache.end())return found->second;
    FlightProperties out{body,rocket,motors,{},{},{}};
    load_body_props(body,out.body);load_rocket(rocket,out.rocket);load_motors(motors,out.motors);
    if(flight_cache.size()>=64)flight_cache.clear();
    return flight_cache.emplace(key,std::move(out)).first->second;
}
V motor_result(V propulsions,V clocks) {
    if(count(propulsions)!=count(clocks))throw PyError("ValueError","One clock per propulsion is required");
    const wtm_motors& motors=prepared(motor_cache,propulsions,load_motors);
    float times[4]={};for(int i=0;i<motors.count;++i)times[i]=at(clocks,i);
    wtm_motor_scalar out=::motor_scalar(&motors,times);check();
    return make_dict({{"thrust",out.thrust},{"mass_lost",out.mass_lost},{"vectoring",out.vectoring}});
}
void load_orientation(V value,wtm_orientation_props& output) {
#define SCALAR(name) output.name=num(value,#name)
    SCALAR(start);SCALAR(duration);SCALAR(final_speed);SCALAR(final_distance);SCALAR(depth_min);SCALAR(angle_to_rate);
#undef SCALAR
    output.use_fins=truth(item(value,"use_fins"));
    read_floats(item(value,"elevation"),output.elevation);read_floats(item(value,"heading"),output.heading);
    read_floats(item(value,"coefficients"),output.coefficients);
}
void load_table(V value,wtm_table& output) {load_rows(value,output.rows,output.count,32,"guidance table too long");}
void load_guidance(V value,wtm_guidance_props& output) {
    output.loft_enabled=truth(item(value,"loft_enabled"));
    output.absolute_altitude=truth(item(value,"absolute_altitude"));
#define SCALAR(name) output.name=num(value,#name)
    SCALAR(loft_elevation);SCALAR(loft_rate_squared);SCALAR(loft_accel);SCALAR(hold_time);SCALAR(hold_elevation);SCALAR(hold_rate);
    SCALAR(elevation_add);SCALAR(angle_gain);SCALAR(omega_gain);SCALAR(course_gain);SCALAR(course_final_angle);
    SCALAR(altitude_diff_gain);SCALAR(altitude_rate_gain);SCALAR(climb_accel_gain);
#undef SCALAR
    read_floats(item(value,"range_elevation"),output.range_elevation);
    read_floats(item(value,"climb_limits"),output.climb_limits);
    load_table(item(value,"time_gain"),output.time_gain);load_table(item(value,"time_to_hit_gain"),output.time_to_hit_gain);
    load_table(item(value,"altitude_table"),output.altitude_table);
}
void load_acceleration(V value,wtm_acceleration_props& output) {
    output.velocity_frame=truth(item(value,"velocity_frame"));output.limit_aoa=truth(item(value,"limit_aoa"));
    output.max_accel=num(value,"max_accel");output.aoa_max=num(value,"aoa_max");output.base_speed_squared=num(value,"base_speed_squared");
    Args schedule=iter(item(value,"schedule")),coefficients=iter(item(value,"coefficients"));
    if(schedule.size()>32||coefficients.size()>16)throw PyError("ValueError","acceleration schedule too long");
    output.schedule_count=static_cast<int>(schedule.size());output.coefficient_count=static_cast<int>(coefficients.size());
    for(size_t i=0;i<coefficients.size();++i) {
        if(count(coefficients[i])!=4)throw PyError("ValueError","Four PID coefficients required");
        read_floats(coefficients[i],output.coefficients[i]);
    }
    for(size_t i=0;i<schedule.size();++i) {
        V index=item(schedule[i],2);
        if(index.kind()!=Kind::Int||index.as_int()<0||index.as_int()>=static_cast<int64_t>(coefficients.size()))
            throw PyError("IndexError","Invalid PID coefficient index");
        read_floats(schedule[i],output.schedule[i]);
    }
}
void load_aero(V value,wtm_aero_props& output) {
    output.mass=num(value,"mass");output.cy=num(value,"cy");output.cy_limit=num(value,"cy_limit");output.side_area=num(value,"side_area");
    load_rows(item(value,"cy_table"),output.cy_table,output.cy_rows,64,"cy table too long");
}
void load_propulsion(V value,wtm_propulsion_control_props& output) {
    Args stages=iter(value);
    if(stages.size()>4)throw PyError("ValueError","Native controller supports at most four stages");
    output.count=static_cast<int>(stages.size());
    for(size_t i=0;i<stages.size();++i) {
        V value=stages[i];auto& stage=output.stages[i];
#define SCALAR(name) stage.name=num(value,#name)
        SCALAR(start);SCALAR(speed_squared);SCALAR(closure_min);SCALAR(tgo_min);SCALAR(cruise);SCALAR(cruise_error_limit);
        SCALAR(base_ias_squared);SCALAR(terminal_tgo);
#undef SCALAR
        stage.use_ias=truth(item(value,"use_ias"));
        read_floats(item(value,"altitude"),stage.altitude);read_floats(item(value,"coefficients"),stage.coefficients);
        read_floats(item(value,"pursuit_closure"),stage.pursuit_closure);
    }
}
V orientation_result(const Args& args,const Kw& kw) {
    Args a=arguments(args,kw,{"p","state","time","range_squared","q","omega","angles","depth","dt"});
    const auto& p=prepared(orientation_cache,a[0],load_orientation);V state=a[1];
    if(count(item(state,"pid"))!=2)throw PyError("ValueError","Two PID records required");
    wtm_orientation_state st{};st.enabled=truth(item(state,"enabled"));
    for(int i=0;i<2;++i)read_floats(item(item(state,"pid"),i),st.pid[i]);
    float angles[2];read_floats(a[6],angles);
    auto out=::orientation_update(&p,&st,number(a[2]),number(a[3]),quat(a[4]),vec3(a[5]),angles,number(a[7]),number(a[8]));check();
    V s=deepcopy(state);if(truth(item(s,"enabled"))&&!st.enabled)dict_set(s,"enabled",false);
    V result=make_dict({{"state",s},{"override",bool(out.override)},{"fins",make_list({0.,0.})},
                       {"auxiliary",make_list({0.,0.})},{"active",false}});
    if(!out.active)return result;
    dict_set(s,"pid",make_list({floats(st.pid[0]),floats(st.pid[1])}));
    dict_set(result,p.use_fins?"fins":"auxiliary",p.use_fins?floats(out.fins):floats(out.auxiliary));
    dict_set(result,"active",true);dict_set(result,"angles",floats(out.angles));
    dict_set(result,"elevation",out.elevation);dict_set(result,"errors",floats(out.errors));
    return result;
}
V guidance_result(const Args& args,const Kw& kw) {
    Args a=arguments(args,kw,{"p","mode","course","los","rate","navigation_velocity","quaternion","velocity",
                             "position","time","time_to_hit","range_squared","height_query"},{{"height_query",V()}});
    const auto& p=prepared(guidance_cache,a[0],load_guidance);
    auto out=::guidance_request(&p,static_cast<int>(a[1].as_int()),static_cast<int>(a[2].as_int()),vec3(a[3]),vec3(a[4]),vec3(a[5]),
         quat(a[6]),vec3(a[7]),vec3(a[8]),number(a[9]),number(a[10]),number(a[11]),!a[12].is_none(),a[12].is_none()?0.:number(a[12]));check();
    if(!out.has_acceleration)return make_dict({{"mode",a[1]},{"course",a[2]},{"gain",out.gain},{"acceleration",make_list()}});
    return make_dict({{"mode",out.mode},{"course",out.course},{"gain",out.gain},{"acceleration",list3(out.acceleration)},
          {"target_pitch",out.target_pitch},{"target_yaw",out.target_yaw},{"angular_command",floats(out.command)}});
}
V acceleration_result(const Args& args,const Kw& kw,V reference) {
    Args a=arguments(args,kw,{"p","aero","motor","state","request","measured","q","velocity","height","time","dt",
                             "environment","matrix_velocity_frame"},{{"environment",V()},{"matrix_velocity_frame",true}},11);
    if(truth(a[11]))return call(reference,args,kw); // Native translated custom-environment path.
    const auto& p=prepared(acceleration_cache,a[0],load_acceleration);
    const wtm_aero_props* aero=a[1].is_none()?nullptr:&prepared(aero_cache,a[1],load_aero);
    wtm_motor_scalar motor{};if(truth(a[2])){motor.thrust=num(a[2],"thrust");motor.mass_lost=num(a[2],"mass_lost");}
    V state=a[3];if(count(state)!=2)throw PyError("ValueError","Two PID records required");
    float pid[16];for(int i=0;i<2;++i)for(int j=0;j<8;++j)pid[i*8+j]=at(item(state,i),j);
    wtm_acceleration_out out{};
    ::acceleration_update(&p,aero,motor,pid,vec3(a[4]),vec3(a[5]),quat(a[6]),vec3(a[7]),number(a[8]),number(a[9]),number(a[10]),truth(a[12]),&out);check();
    V frame=list4(out.frame);
    if(!out.has_fins)return make_dict({{"frame",frame},{"fins",V()},{"state",state}});
    Args states;for(int i=0;i<2;++i){Args row;for(int j=0;j<8;++j)row.push_back(V(pid[i*8+j]));states.push_back(make_list(std::move(row)));}
    return make_dict({{"frame",frame},{"fins",floats(out.fins)},{"state",make_list(std::move(states))},
          {"errors",floats(out.errors)},{"limited_request",floats(out.limited)},{"aoa_limit",out.has_aoa_limit?V(out.aoa_limit):V()},
          {"scale",out.scale},{"scale_squared",out.scale_squared},{"coefficients",floats(out.coefficients)}});
}
V propulsion_result(const Args& args,const Kw& kw,V reference) {
    Args a=arguments(args,kw,{"p","state","time","position","velocity","tgo","closure","dt","environment"},{{"environment",V()}},8);
    if(truth(a[8]))return call(reference,args,kw); // Native translated custom-environment path.
    V stages=a[0],state=a[1];size_t length=count(stages);
    if(length>4)throw PyError("ValueError","Native controller supports at most four stages");
    if(count(item(state,"stages"))!=4||count(item(state,"pid"))!=8)
        throw PyError("ValueError","Four enums and one eight-float shared PID required");
    V s=deepcopy(state);if(!length)return make_dict({{"state",s},{"factors",make_list({1.,1.,1.,1.})}});
    const auto& p=prepared(propulsion_cache,stages,load_propulsion);
    wtm_propulsion_control_state st{};bool cruising=false;
    for(int i=0;i<4;++i){st.stages[i]=static_cast<int>(item(item(state,"stages"),i).as_int());if(i<static_cast<int>(length)&&st.stages[i]==2)cruising=true;}
    read_floats(item(state,"pid"),st.pid);float factors[4];
    ::propulsion_control_update(&p,&st,number(a[2]),vec3(a[3]),vec3(a[4]),number(a[5]),number(a[6]),number(a[7]),factors);check();
    if(cruising)dict_set(s,"pid",floats(st.pid));
    dict_set(s,"stages",make_list({st.stages[0],st.stages[1],st.stages[2],st.stages[3]}));
    return make_dict({{"state",s},{"factors",floats(factors)}});
}
V aero_eval(const wtm_aero_eval& e) {
    return make_dict({{"drag",list3(e.drag)},{"lift",list3(e.lift)},{"force",list3(e.force)},
                      {"cosine",e.cosine},{"cd",e.cd},{"cy",e.cy}});
}
V forces_result(const Args& args,const Kw& kw,V reference) {
    V zero=make_tuple({0.,0.,0.});
    Args a=arguments(args,kw,{"props","height","velocity","q","omega","wind","fins","additional_cx","additional_lever",
         "dt","torque","force","mass_lost","gravity","use_cxi","mass_term","angular_environment","perturbation","body_random"},
         {{"wind",zero},{"fins",make_tuple({0.,0.})},{"additional_cx",0.},{"additional_lever",0.},{"dt",1./48.},
          {"torque",zero},{"force",zero},{"mass_lost",0.},{"gravity",true},{"use_cxi",true},{"mass_term",0.},
          {"angular_environment",zero},{"perturbation",0.},{"body_random",0.}},5);
    auto zero_vector=[](V v){Args values=iter(v);if(values.size()!=3)return false;
        for(V value:values){double number=value.as_double();if(number!=0.||std::signbit(number))return false;}return true;};
    if(!zero_vector(a[5])||a[7].as_double()!=0.||a[8].as_double()!=0.||!truth(a[13])||!truth(a[14])||
         a[15].as_double()!=0.||!zero_vector(a[16]))return call(reference,args,kw);
    const auto& p=prepared(body_props_cache,a[0],load_body_props);wtm_forces f{};
    float amplitude=number(a[17]),random=number(a[18]);
    aero_forces(&p,number(a[1]),vec3(a[2]),quat(a[3]),vec3(a[4]),at(a[6],0),at(a[6],1),number(a[9]),
                vec3(a[11]),vec3(a[10]),number(a[12]),amplitude,random,&f);check();
    wtm_perturb perturb=perturbation_coefficients(amplitude,random);check();
    float offset=fmul(fmul(perturb.lever_fraction,random),p.stabilizer_arm);
    float cosine_offset=fmul(perturb.cosine,offset),sine_offset=fmul(offset,perturb.sine);
    return make_dict({{"frame",make_list({list3(f.frame[0]),list3(f.frame[1]),list3(f.frame[2])})},
       {"axes",make_list({list3(f.axes[0]),list3(f.axes[1]),list3(f.axes[2])})},{"cm_speed",f.cm_speed},
       {"mach",f.mach},{"pressure",f.pressure},{"local_flow",list3(f.local_flow)},{"flow",list3(f.flow)},
       {"baseline",aero_eval(f.baseline)},{"fin_active",bool(f.fin_active)},{"fin_limited",bool(f.fin_limited)},
       {"deflection",floats(f.deflection)},{"fin_flow",list3(f.fin_flow)},{"steering",aero_eval(f.steering)},
       {"moment",list3(f.moment)},{"damping",floats(f.damping)},
       {"damping_clipped",make_list({bool(f.damping_clipped[0]),bool(f.damping_clipped[1]),bool(f.damping_clipped[2])})},
       {"angular_acceleration_before_environment",list3(f.angular_acceleration_before_environment)},
       {"angular_acceleration",list3(f.angular_acceleration)},
       {"acceleration",make_list({f.acceleration[0],f.acceleration[1],f.acceleration[2]})},{"effective_mass",f.effective_mass},
       {"perturbation",make_dict({{"force_scale",perturb.force_scale},{"angle",perturb.angle},{"cosine",perturb.cosine},
                                 {"sine",perturb.sine},{"lever_fraction",perturb.lever_fraction}})},
       {"perturbed_lever",make_list({V(fadd(p.stabilizer_arm,0.0f)),V(-sine_offset),V(-cosine_offset)})}});
}
V flight_result(const Args& args,const Kw& kw,V default_guidance) {
    Args a=arguments(args,kw,{"p","state","targets","dt","guidance_step"},{{"guidance_step",default_guidance}},4);
    V p=a[0];float dt=number(a[3]);
    if(!std::isfinite(dt)||dt<=0.)throw PyError("ValueError","Positive finite caller interval required");
    if(item(p,"precision").as_double()<=0.)throw PyError("ValueError","Positive body substep count required");
    V matrix=dict_get(p,"matrix_velocity_frame",true);
    if(matrix.kind()!=Kind::Bool)throw PyError("ValueError","matrix_velocity_frame must be an explicit boolean");
    V state=deepcopy(a[1]),body=item(state,"body"),old=packet(body),controls=item(state,"controls");
    if(truth(item(body,"water"))||num(body,"immersion")!=0.)throw PyError("NotImplementedError","Pure-air composition only");
    if(count(item(body,"clocks"))!=4||count(controls)!=8)
        throw PyError("ValueError","Adapter requires four stored clocks and four factors");
    const auto& props=flight_properties(p);
    int precision=static_cast<int>(item(p,"precision").as_int());
    wtm_flight_props fp{&props.body,&props.rocket,&props.motors,precision,num(p,"end_speed")};
    wtm_body b;load_body(body,b);float c[8];read_floats(controls,c);
    wtm_body_step_out out{};std::vector<wtm_substep> steps(precision);
    body_advance(&fp,&b,c,dt,&out,steps.data());check();
    dict_set(body,"seed",b.seed);dict_set(body,"surface_depth",b.surface_depth);
    Args substeps;substeps.reserve(precision);
    for(const auto& step:steps) {
        V entry=make_dict();dict_merge(entry,body);store_body(entry,step.state);
        substeps.push_back(make_dict({{"state",entry},
            {"acceleration",make_list({step.acceleration[0],step.acceleration[1],step.acceleration[2]})},
            {"angular_acceleration",list3(step.angular_acceleration)}}));
    }
    store_body(body,b);V pre_cap=packet(body);
    float launch=number(dict_get(state,"launch_time",0.));
    V guidance=call(a[4],{item(p,"guidance"),item(state,"guidance"),old,pre_cap,
                    V(fsub(out.target_time,launch)),item(body,"clocks"),a[2]},
                   {{"context_value",item(body,"surface_depth")},{"matrix_velocity_frame",matrix}});
    dict_set(state,"guidance",item(guidance,"state"));V manager=item(item(state,"guidance"),"manager");
    dict_set(state,"controls",binary("Add",binary("Add",item(manager,"fins"),item(manager,"orientation_auxiliary")),item(manager,"factors")));
    dict_set(body,"velocity",list3(::cap_velocity(vec3(item(body,"velocity")),num(p,"end_speed"))));check();
    V propulsion=make_dict({{"world_force",list3(out.propulsion.world_force)},
       {"body_torque",list3(out.propulsion.body_torque)},{"mass_lost",out.propulsion.mass_lost},
       {"clock_rates",floats(out.propulsion.clock_rates)},{"seed",out.propulsion.seed}});
    return make_dict({{"state",state},{"propulsion",propulsion},{"substeps",make_list(std::move(substeps))},
                      {"pre_cap",pre_cap},{"guidance",guidance}});
}
}

void clear_kernel_caches() {
    flight_cache.clear();motor_cache.clear();orientation_cache.clear();guidance_cache.clear();
    acceleration_cache.clear();aero_cache.clear();propulsion_cache.clear();body_props_cache.clear();
}

void apply_kernel_overrides(const std::string& name,E env) {
    if(env->values.find("_native_kernel_overrides")!=env->values.end())return;
    if(name=="kernels") {
        expose(env,"f32",[](const Args& a,const Kw& k){auto b=arguments(a,k,{"x"});return V(number(b[0]));});
        get(env,"f32").node()->unary_fast=fast_f32;
        for(const char* op:{"add","sub","mul","div","safe_div"}) {
            std::string operation=op;
            expose(env,op,[operation](const Args& a,const Kw& k){
                auto b=arguments(a,k,{"a","b"});float y=number(b[1]);
                if(operation=="safe_div"&&!(fabsf(y)>F(4e-19)))return V(0.);
                float x=number(b[0]),result;
                if(operation=="add")result=fadd(x,y);else if(operation=="sub")result=fsub(x,y);
                else if(operation=="mul")result=fmul(x,y);else if(operation=="div")result=fdiv(x,y);
                else result=::safe_div(x,y);check();return V(result);
            });
            get(env,op).node()->binary_fast=operation=="add"?fast_arithmetic<0>:operation=="sub"?fast_arithmetic<1>:
                operation=="mul"?fast_arithmetic<2>:operation=="div"?fast_arithmetic<3>:fast_arithmetic<4>;
        }
        expose(env,"atmosphere",[](const Args& a,const Kw& k){auto b=arguments(a,k,{"height"});
            wtm_air out=::atmosphere(number(b[0]));check();
            return make_dict({{"density",out.density},{"sound_speed",out.sound_speed},{"pressure",out.pressure}});});
        expose(env,"drag_mach",[](const Args& a,const Kw& k){auto b=arguments(a,k,{"mach"});
            float out=::drag_mach(number(b[0]));check();return V(out);});
        expose(env,"motor_scalar",[](const Args& a,const Kw& k){auto b=arguments(a,k,{"propulsions","clocks"});return motor_result(b[0],b[1]);});
        expose(env,"pressure_multiplier",[](const Args& a,const Kw& k){auto b=arguments(a,k,{"rocket","height"});
            wtm_rocket rocket;load_rocket(b[0],rocket);float result=::pressure_multiplier(&rocket,number(b[1]));check();return V(result);});
    } else if(name=="native_atan2f") {
        expose(env,"atan2f",[](const Args& a,const Kw& k){auto b=arguments(a,k,{"y","x"});
            float out=atan2f_native(number(b[0]),number(b[1]));check();return V(out);});
    } else if(name=="native_tanf") {
        expose(env,"tanf",[](const Args& a,const Kw& k){auto b=arguments(a,k,{"x"});
            float out=tanf_native(number(b[0]));check();return V(out);});
    } else if(name=="body_integration") {
        expose(env,"half_angle_trig",[](const Args& a,const Kw& k){auto b=arguments(a,k,{"increment"});
            wtm_trig out=::half_angle_trig(number(b[0]));check();
            return make_dict({{"sine",out.sine},{"cosine",out.cosine},{"quadrant",V(static_cast<int64_t>(out.quadrant))},{"reduced",out.reduced}});});
    } else if(name=="motor_vector") {
        expose(env,"rotate_thrust",[](const Args& a,const Kw& k){auto b=arguments(a,k,{"quaternion","thrust"});
            v3 out=::rotate_thrust(quat(b[0]),vec3(b[1]));check();return list3(out);});
    } else if(name=="aero_vectors") {
        V reference=get(env,"forces");
        expose(env,"forces",[reference](const Args& a,const Kw& k){return forces_result(a,k,reference);});
    } else if(name=="orientation_control") {
        expose(env,"update",orientation_result);
    } else if(name=="guidance_request") {
        expose(env,"request",guidance_result);
    } else if(name=="acceleration_control") {
        V reference=get(env,"update");
        expose(env,"update",[reference](const Args& a,const Kw& k){return acceleration_result(a,k,reference);});
    } else if(name=="propulsion_control") {
        V reference=get(env,"update");
        expose(env,"update",[reference](const Args& a,const Kw& k){return propulsion_result(a,k,reference);});
    } else if(name=="optical_geometry") {
        expose(env,"boresight",[](const Args& a,const Kw& k){auto b=arguments(a,k,{"quaternion","angles"});float angles[2];read_floats(b[1],angles);
            v3 result=::boresight(quat(b[0]),angles);check();return list3(result);});
    } else if(name=="shared_seeker") {
        expose(env,"normalize",[](const Args& a,const Kw& k){auto b=arguments(a,k,{"v","phase"});std::string phase=b[1].as_string();
            int p=phase=="predict"?NORM_PREDICT:phase=="correct"?NORM_CORRECT:phase=="coast"?NORM_COAST:NORM_OTHER;
            v3 result=::seeker_normalize(vec3(b[0]),p);check();return list3(result);});
    } else if(name=="control_frame" || name=="alternate_control_frame") {
        bool alternate=name=="alternate_control_frame";
        expose(env,"frame",[alternate](const Args& a,const Kw& k){
            Args b=alternate?arguments(a,k,{"quaternion","velocity","velocity_reference"},{{"velocity_reference",true}}):
               arguments(a,k,{"quaternion","velocity","velocity_reference","matrix"},{{"velocity_reference",true},{"matrix",true}},3);
            q4 result=::control_frame(quat(b[0]),vec3(b[1]),truth(b[2]),alternate?false:truth(b[3]));check();return list4(result);});
    } else if(name=="optical_point_flight") {
        V guidance=get(env,"guidance_update");
        expose(env,"update",[guidance](const Args& a,const Kw& k){return flight_result(a,k,guidance);});
        expose(env,"cap_velocity",[](const Args& a,const Kw& k){auto b=arguments(a,k,{"velocity","end_speed"});
            v3 out=::cap_velocity(vec3(b[0]),number(b[1]));check();return list3(out);});
    } else if(name=="state_binary32") {
        V reference=get(env,"validate_flight_numbers");
        expose(env,"validate_flight_numbers",[reference](const Args& a,const Kw& k){
            auto args=arguments(a,k,{"state"});
            if(!valid_numbers(args[0],NumberPath::Root,false))return call(reference,a,k);
            return V();
        });
    } else if(name=="flight_session") {
        V reference=get(env,"finite");
        expose(env,"finite",[reference](const Args& a,const Kw& k){
            auto args=arguments(a,k,{"value","path"},{{"path","state"}});
            if(args[1].kind()!=Kind::String || !valid_numbers(args[0],NumberPath::Other,true))return call(reference,a,k);
            return V();
        });
        bound_invocation(env,"FlightSession","step");
    } else if(name=="frame_session") {
        bound_invocation(env,"FrameSession","advance");
    } else if(name=="interaction_session") {
        bound_invocation(env,"InteractionSession","advance");
    } else if(name=="missile_fast") {
        bound_invocation(env,"FastFrameSession","advance");
        bound_invocation(env,"FastInteractionSession","advance");
    } else if(name=="missile_envelope") {
        install_typed_evaluate(env);
    } else return;
    set(env,"_native_kernel_overrides",true);
}
void install_kernel_overrides() {
    for(const char* name:{"kernels","native_atan2f","native_tanf","body_integration","motor_vector","aero_vectors",
                        "orientation_control","guidance_request","acceleration_control","propulsion_control",
                        "optical_geometry","shared_seeker","control_frame","alternate_control_frame","optical_point_flight",
                        "state_binary32","flight_session","frame_session","interaction_session","missile_fast"})
        apply_kernel_overrides(name,module_env(name));
}
}
