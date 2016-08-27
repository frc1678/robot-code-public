#include "timer.h"
#include "timing_utils.h"

namespace muan {

Timer::Timer() { Start(); }

void Timer::Start() { start_ = now(); }

muan::units::Seconds Timer::Reset() {
  using namespace muan::units;
  Seconds n = now();
  std::swap(start_, n);
  return start_ - n;
}

muan::units::Seconds Timer::Get() { return now() - start_; }
}
