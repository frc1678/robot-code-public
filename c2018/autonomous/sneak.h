#ifndef C2018_AUTONOMOUS_SWITCH_H_
#define C2018_AUTONOMOUS_SWITCH_H_

#include "c2018/autonomous/autonomous_base.h"
#include "muan/logging/logger.h"

namespace c2018 {
namespace autonomous {

class Sneak : public c2018::autonomous::AutonomousBase {
 public:
  void SneakLeft();
  void SneakRight();
};

}  // namespace autonomous
}  // namespace c2018

#endif  // C2018_AUTONOMOUS_SWITCH_H_
