#ifndef C2019_AUTONOMOUS_TEST_AUTO_H_
#define C2019_AUTONOMOUS_TEST_AUTO_H_

#include "muan/logging/logger.h"
#include "muan/units/units.h"
#include "c2019/autonomous/autonomous_base.h"

namespace c2019 {
namespace autonomous {

class TestAuto : public c2019::autonomous::AutonomousBase {
 public:
  void Run();
};

}  // namespace autonomous
}  // namespace c2019

#endif  // C2019_AUTONOMOUS_TEST_AUTO_H_
