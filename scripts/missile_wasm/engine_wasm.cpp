#include "engine_stdlib.h"
#include "typed_engine.h"
#include <emscripten/emscripten.h>
#include <malloc.h>
#include <string>
#include "live_build.h"

void register_generated_modules();
static std::string error, info;
static bool initialized=false;
extern "C" {
EMSCRIPTEN_KEEPALIVE int engine_init(const char* root) {
    try {if(initialized)throw std::runtime_error("Engine already initialized");
        nt::register_native_stdlib(root);register_generated_modules();nt::import_module("missile_worker");initialized=true;return 0;
    }catch(const std::exception& e){error=e.what();}catch(...){error="Engine initialization failed";}return -1;
}
EMSCRIPTEN_KEEPALIVE const char* engine_info(){
    try{info=nt::json_dump(nt::make_dict({{"native_signature",NATIVE_SIGNATURE},{"wasm_fingerprint",WASM_FINGERPRINT},
        {"emsdk",EMSDK_VERSION},{"state_version",nt::LIVE_STATE_VERSION},{"state_size",nt::LIVE_STATE_SIZE},
        {"open_handles",int(nt::live_open_handles())},{"heap_bytes",double(mallinfo().uordblks)}}),false,false);return info.c_str();
    }catch(const std::exception& e){error=e.what();}catch(...){error="Engine info failed";}return nullptr;
}
EMSCRIPTEN_KEEPALIVE int live_start(const char* request){
    try{if(!initialized)throw std::runtime_error("Engine is not initialized");nt::InvocationGuard scope;
        int h=nt::live_start_native(nt::json_parse(request));if(h<0)error=nt::live_last_error();return h;
    }catch(const std::exception& e){error=e.what();}catch(...){error="Live start failed";}return -1;
}
EMSCRIPTEN_KEEPALIVE int live_step(int h,int az,int el,int roll,int stop){
    int result=nt::live_step_native(h,az,el,roll,stop!=0);if(result<0)error=nt::live_last_error();return result;
}
EMSCRIPTEN_KEEPALIVE const double* live_state_ptr(){return nt::live_state_native();}
EMSCRIPTEN_KEEPALIVE const char* live_result(int h){
    try{nt::InvocationGuard scope;const char* result=nt::live_result_native(h);if(!result)error=nt::live_last_error();return result;
    }catch(const std::exception& e){error=e.what();}catch(...){error="Live result failed";}return nullptr;
}
EMSCRIPTEN_KEEPALIVE void live_free(int h){nt::live_free_native(h);}
EMSCRIPTEN_KEEPALIVE const char* last_error(){return error.c_str();}
}
