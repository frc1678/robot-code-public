#ifndef O2018_AUTONOMOUS_TEST_AUTO_H_
#define O2018_AUTONOMOUS_TEST_AUTO_H_

#include "muan/logging/logger.h"
#include "muan/units/units.h"
#include "o2018/autonomous/autonomous_base.h"

namespace o2018 {
namespace autonomous {

class TestAuto : public o2018::autonomous::AutonomousBase {
 public:
  void LeftSwitch();
  void RightSwitch();
};

}  // namespace autonomous
}  // namespace o2018

#endif  // O2018_AUTONOMOUS_TEST_AUTO_H_
