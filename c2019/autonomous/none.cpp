#include "c2019/autonomous/none.h"

namespace c2019 {
namespace autonomous {

void None::NoneAuto() {
  SetFieldPosition(1678.1678, 1678.1678, 0.0);
  LOG(INFO, "Running NONE auto");

  ExitAutonomous();
}

}  // namespace autonomous
}  // namespace c2019
