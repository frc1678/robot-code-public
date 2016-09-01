#include "timing_utils.h"

namespace muan {

void sleep_for(muan::units::Time t) {
  using namespace muan::units;
  std::this_thread::sleep_for(std::chrono::milliseconds((int)convert(t, ms)));
}

muan::units::Time now() {
  using namespace muan::units;
  using namespace std::chrono;
  auto since_epoch = system_clock::now().time_since_epoch();
  return duration_cast<microseconds>(since_epoch).count() * us;
}

void sleep_until(muan::units::Time t) {
  using namespace muan::units;
  using namespace std::chrono;
  auto as_time_point = time_point<system_clock>(
      microseconds(static_cast<long long>(convert(t, us))));
  std::this_thread::sleep_until(as_time_point);
}
}
