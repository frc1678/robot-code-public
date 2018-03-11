#include "c2018/autonomous/switch_and_scale.h"

namespace c2018 {
namespace autonomous {

void SwitchAndScale::LeftSwitchLeftScale() {
  // LL - Switch is left, scale is left
  LOG(INFO, "Running LEFT SWITCH LEFT SCALE auto");
  // auto_running = true;
}

void SwitchAndScale::RightSwitchRightScale() {
  // RR - Switch is right, scale is right
  LOG(INFO, "Running RIGHT SWITCH RIGHT SCALE auto");
  // DoAuto();
}

void SwitchAndScale::RightSwitchLeftScale() {
  // RL - Switch is right, scale is left
  LOG(INFO, "Running RIGHT SWITCH LEFT SCALE auto");
  // Write autos here!
}

void SwitchAndScale::LeftSwitchRightScale() {
  // Switch is left, scale is right
  LOG(INFO, "Running LEFT SWITCH RIGHT SCALE auto");
  // 404 Auto not found
}

}  // namespace autonomous
}  // namespace c2018
