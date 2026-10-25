#pragma once
#include "runtime.h"
#include <functional>
namespace nt {
void register_native_stdlib(const std::string& root);
std::string engine_source_path(const std::string& relative);
extern thread_local std::function<V(V)> envelope_yield;
}
