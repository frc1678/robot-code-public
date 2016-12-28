#include "timer.h"
#include "timing_utils.h"

namespace muan {
namespace utils {

Timer::Timer() { Start(); }

void Timer::Start() { start_ = now(); }

muan::units::Time Timer::Reset() {
  using namespace muan::units;
  Time n = now();
  std::swap(start_, n);
  return start_ - n;
}

muan::units::Time Timer::Get() { return now() - start_; }

}  // utils
}  // muan
