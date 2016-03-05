#include "timer.h"
#include "timing_utils.h"
#include "muan/unitscpp/unitscpp.h"

namespace muan {

Timer::Timer() { Start(); }

void Timer::Start() { start_ = now(); }

void Timer::Reset() { start_ = now(); }

Time Timer::Get() { return now() - start_; }
}
