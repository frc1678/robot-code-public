#include "timing_utils.h"
#include "third_party/unitscpp/unitscpp.h"
#include <cstdint>

namespace muan {

using std::chrono::system_clock;
using std::chrono::duration_cast;
using std::chrono::microseconds;
using std::chrono::time_point;

void sleep_for(Time t) {
  std::this_thread::sleep_for(
      std::chrono::milliseconds(static_cast<int>(t.to(ms))));
}

Time now() {
  auto since_epoch = system_clock::now().time_since_epoch();
  return duration_cast<microseconds>(since_epoch).count() * us;
}

void sleep_until(Time t) {
  auto as_time_point =
      time_point<system_clock>(microseconds(static_cast<uint64_t>(t.to(us))));
  std::this_thread::sleep_until(as_time_point);
}

}  // namespace muan
