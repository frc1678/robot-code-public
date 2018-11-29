#ifndef O2018_AUTONOMOUS_DRIVE_STRAIGHT_H_
#define O2018_AUTONOMOUS_DRIVE_STRAIGHT_H_

#include "muan/logging/logger.h"
#include "o2018/autonomous/autonomous_base.h"

namespace o2018 {
namespace autonomous {

class DriveStraight : public o2018::autonomous::AutonomousBase {
 public:
  void Drive();
};

}  // namespace autonomous
}  // namespace o2018

#endif  // O2018_AUTONOMOUS_DRIVE_STRAIGHT_H_
