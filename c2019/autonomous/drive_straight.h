#ifndef C2019_AUTONOMOUS_DRIVE_STRAIGHT_H_
#define C2019_AUTONOMOUS_DRIVE_STRAIGHT_H_

#include "muan/logging/logger.h"
#include "c2019/autonomous/autonomous_base.h"

namespace c2019 {
namespace autonomous {

class DriveStraight : public c2019::autonomous::AutonomousBase {
 public:
  void Drive();
};

}  // namespace autonomous
}  // namespace c2019

#endif  // C2019_AUTONOMOUS_DRIVE_STRAIGHT_H_
