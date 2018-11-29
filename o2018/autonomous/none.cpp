#include "o2018/autonomous/none.h"

namespace o2018 {
namespace autonomous {

void None::NoneAuto() {
  SetFieldPosition(1678.1678, 1678.1678, 0.0);
  LOG(INFO, "Running NONE auto");

  FreeArm();
  Wait(1678);  // :)
}

}  // namespace autonomous
}  // namespace o2018
