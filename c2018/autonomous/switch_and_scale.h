#ifndef C2018_AUTONOMOUS_SWITCH_AND_SCALE_H_
#define C2018_AUTONOMOUS_SWITCH_AND_SCALE_H_

#include "c2018/autonomous/autonomous_base.h"
#include "muan/logging/logger.h"

namespace c2018 {
namespace autonomous {

class SwitchAndScale : public c2018::autonomous::AutonomousBase {
 public:
  void RightRightSwitch();
  void RightLeftSwitch();
  void LeftRightSwitch();
  void LeftLeftSwitch();
};

}  // namespace autonomous
}  // namespace c2018

#endif  // C2018_AUTONOMOUS_SWITCH_AND_SCALE_H_
