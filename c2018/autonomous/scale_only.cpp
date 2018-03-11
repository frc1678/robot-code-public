#include "c2018/autonomous/scale_only.h"

namespace c2018 {
namespace autonomous {

void ScaleOnly::LeftScale() {
  // XL - Scale is left
  LOG(INFO, "Running LEFT SCALE ONLY auto");
  // auto_running = true;
}

void ScaleOnly::RightScale() {
  // XR - Scale is right
  LOG(INFO, "Running RIGHT SCALE ONLY auto");
  // DoAuto();
}

}  // namespace autonomous
}  // namespace c2018
