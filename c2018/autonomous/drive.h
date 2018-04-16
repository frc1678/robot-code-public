#ifndef C2018_AUTONOMOUS_DRIVE_H_
#define C2018_AUTONOMOUS_DRIVE_H_

#include "c2018/autonomous/autonomous_base.h"
#include "muan/logging/logger.h"

namespace c2018 {
namespace autonomous {

class Drive : public c2018::autonomous::AutonomousBase {
 public:
  void DriveBackwards();
};

}  // namespace autonomous
}  // namespace c2018

#endif  // C2018_AUTONOMOUS_DRIVE_H_
