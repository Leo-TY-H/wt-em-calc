#pragma once
#include "runtime.h"

namespace nt {
// Replace missile_envelope.evaluate with the typed evaluation loop. The
// translated function remains the fallback for every case the typed port
// declines (WT_MISSILE_TYPED=0 disables it; =compare checks each sample).
void install_typed_evaluate(E env);
// Replace missile_worker.simulate with the typed simulator loop (same switch;
// =compare runs both and reports the first differing trajectory row).
void install_typed_simulate(E env);
}
