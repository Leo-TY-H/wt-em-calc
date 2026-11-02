#pragma once
#include "runtime.h"

namespace nt {
constexpr int LIVE_STATE_VERSION=1;
constexpr int LIVE_STATE_SIZE=32;
int live_start_native(const V& request);
int live_step_native(int h,int az,int el,int roll,bool stop);
const char* live_result_native(int h);
void live_free_native(int h);
const char* live_last_error();
const double* live_state_native();
size_t live_open_handles();

// Replace missile_envelope.evaluate with the typed evaluation loop. The
// translated function remains the fallback for every case the typed port
// declines (WT_MISSILE_TYPED=0 disables it; =compare checks each sample).
void install_typed_evaluate(E env);
// Replace missile_worker.simulate with the typed simulator loop (same switch;
// =compare runs both and reports the first differing trajectory row).
void install_typed_simulate(E env);
}
