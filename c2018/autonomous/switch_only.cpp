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
  // DoAuto();
}

}  // namespace autonomous
}  // namespace c2018
