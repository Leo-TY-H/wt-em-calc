#pragma once
#include "runtime.h"

namespace nt {
// Called after a module's native initialization and before import returns.
void apply_kernel_overrides(const std::string& module_name, E env);
// Refresh overrides for modules which have already been imported.
void install_kernel_overrides();
// Clear immutable property conversions in the calling native worker thread.
void clear_kernel_caches();
}
