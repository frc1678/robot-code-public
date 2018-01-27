#ifndef O2017_CITRUS_ROBOT_MAIN_H_
#define O2017_CITRUS_ROBOT_MAIN_H_

#include "muan/teleop/joystick.h"
#include "muan/wpilib/ds_sender.h"
#include "o2017/queue_manager/queue_manager.h"

namespace o2017 {

namespace citrus_robot {

enum class AutoState {
  kInit,
  kDriveForwards,
  kGearEject,
  kDriveBackwards,
  kDone
};

class CitrusRobot {
 public:
  CitrusRobot();

  void RunAutonomous();

  // Call this to update at ~50hz (DS update rate)
  void Update();
  void UpdateAutonomous();

 private:
  void SetDriveGoal(double dist);
  bool DrivingDone() const;

  muan::teleop::Joystick throttle_, wheel_;
  muan::teleop::Joystick gamepad_;

  bool high_gear_;
  bool climbing_ = false;
  muan::teleop::Button *shifting_high_, *shifting_low_;
  muan::teleop::Button* quickturn_;

  muan::teleop::Button* hp_gear_;
  muan::teleop::Button* climber_;

  muan::wpilib::DriverStationSender ds_sender_;

  frc971::control_loops::drivetrain::GoalProto drivetrain_goal_;

  AutoState auto_state_ = AutoState::kInit;
  aos::monotonic_clock::time_point start_;

  void SendDrivetrainMessage();
};

}  // namespace citrus_robot

}  // namespace o2017

#endif  // O2017_CITRUS_ROBOT_MAIN_H_
