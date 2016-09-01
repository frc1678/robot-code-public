#include "timer.h"
#include "third_party/unitscpp/unitscpp.h"
#include "timing_utils.h"

namespace muan {

Timer::Timer() { Start(); }

void Timer::Start() { start_ = now(); }

Time Timer::Reset() {
  Time n = now();
  std::swap(start_, n);
  return start_ - n;
}

Time Timer::Get() { return now() - start_; }

}  // namespace muan
