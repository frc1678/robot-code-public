#include "timer.h"
#include "timing_utils.h"

namespace muan {
using namespace muan::units;

Timer::Timer() { Start(); }

void Timer::Start() { start_ = now(); }

Seconds Timer::Reset() {
  Seconds n = now();
  std::swap(start_, n);
  return start_ - n;
}

Seconds Timer::Get() { return now() - start_; }
}
