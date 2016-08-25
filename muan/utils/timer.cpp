#include "timer.h"
#include "timing_utils.h"
#include "unitscpp/unitscpp.h"

namespace muan {

Timer::Timer() { Start(); }

void Timer::Start() { start_ = now(); }

Time Timer::Reset() {
  Time n = now();
  std::swap(start_, n);
  return start_ - n;
}

Time Timer::Get() { return now() - start_; }
}
