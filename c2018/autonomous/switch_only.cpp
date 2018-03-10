#include "c2018/autonomous/switch_only.h"

namespace c2018 {
namespace autonomous {

void SwitchOnly::LeftSwitch() {
  // L - Switch is left
  LOG(INFO, "Running LEFT SWITCH ONLY auto");
  // auto_running = true;
}

void SwitchOnly::RightSwitch() {
  // R - Switch is right
  LOG(INFO, "Running RIGHT SWITCH ONLY auto");
  StartDrivePath(2.4, -1.2, 0);
  WaitUntilDriveComplete();
  StartDrivePath(1.2, 1.2, -0.7);
  WaitUntilDriveComplete();

  // DoAuto();
}

}  // namespace autonomous
}  // namespace c2018
