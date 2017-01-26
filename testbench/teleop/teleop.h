#ifndef TESTBENCH_TELEOP_TELEOP_H_
#define TESTBENCH_TELEOP_TELEOP_H_

#include "muan/actions/drivetrain_action.h"
#include "muan/teleop/joystick.h"

namespace testbench {
namespace teleop {

class Teleop {
 public:
  Teleop();

  // Call this to update at ~50hz (DS update rate)
  void Update();

 private:
  muan::actions::DrivetrainProperties properties_;
  muan::teleop::Joystick throttle_, wheel_;

  bool high_gear_;
  muan::teleop::Button *shifting_high_, *shifting_low_;
  muan::teleop::Button* quickturn_;
  muan::teleop::Button* drive_profile_;

  bool running_action_{false};
  muan::actions::DriveSCurveAction action_;

  void SendDSMessage();
  void SendDrivetrainMessage();
};

}  // namespace teleop
}  // namespace testbench

#endif  // TESTBENCH_TELEOP_TELEOP_H_
