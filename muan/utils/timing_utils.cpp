#include "timing_utils.h"
#include <cstdint>

namespace muan {
namespace utils {

using std::chrono::system_clock;
using std::chrono::duration_cast;
using std::chrono::microseconds;
using std::chrono::time_point;

void sleep_for(muan::units::Time t) {
  using namespace muan::units;
  std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<uint64_t>(convert(t, ms))));
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
  auto as_time_point = time_point<system_clock>(microseconds(static_cast<uint64_t>(convert(t, us))));
  std::this_thread::sleep_until(as_time_point);
}

}  // utils
}  // muan
