#include "generic_robot/citrus_robot/main.h"
#include "WPILib.h"
#include "generic_robot/queue_manager/queue_manager.h"
#include "muan/wpilib/queue_types.h"
#include "third_party/aos/common/util/phased_loop.h"

namespace generic_robot {
namespace citrus_robot {

CitrusRobot::CitrusRobot()
    : throttle_{1, generic_robot::QueueManager::GetInstance()
                       ->throttle_status_queue()},
      wheel_{0,
             generic_robot::QueueManager::GetInstance()->wheel_status_queue()},
      gamepad_{2, generic_robot::QueueManager::GetInstance()
                      ->manipulator_status_queue()},
      ds_sender_{QueueManager::GetInstance()->driver_station_queue()},
      ds_reader_{
          QueueManager::GetInstance()->driver_station_queue()->MakeReader()} {
  shifting_low_ = throttle_.MakeButton(4);
  shifting_high_ = throttle_.MakeButton(5);
  quickturn_ = wheel_.MakeButton(5);
}

void CitrusRobot::operator()() {
  aos::time::PhasedLoop phased_loop(std::chrono::milliseconds(20));
  aos::SetCurrentThreadRealtimePriority(10);
  aos::SetCurrentThreadName("CitrusRobot");

  running_ = true;
  while (running_) {
    throttle_.Update();
    wheel_.Update();
    gamepad_.Update();
    Update();
    phased_loop.SleepUntilNext();
  }
}

void CitrusRobot::Stop() { running_ = false; }

void CitrusRobot::Update() {
  auto maybe_ds_status = ds_reader_.ReadLastMessage();
  if (maybe_ds_status) {
    auto ds_status = maybe_ds_status.value();
    if (ds_status->mode() == RobotMode::AUTONOMOUS) {
      // Put running auto here
    } else if (ds_status->mode() == RobotMode::TELEOP) {
      // Stop running auto here

      // Put any rumble controls here

      SendDrivetrainMessage();
      SendSuperstructureMessage();
    }
  }
  ds_sender_.Send();
}

void CitrusRobot::SendSuperstructureMessage() {
  // Put superstructure goals here
}

void CitrusRobot::SendDrivetrainMessage() {
  frc971::control_loops::drivetrain::GoalProto drivetrain_goal;

  double throttle = -throttle_.wpilib_joystick()->GetRawAxis(1);
  double wheel = -wheel_.wpilib_joystick()->GetRawAxis(0);
  bool quickturn = quickturn_->is_pressed();

  if (shifting_high_->was_clicked()) {
    high_gear_ = true;
  }
  if (shifting_low_->was_clicked()) {
    high_gear_ = false;
  }

  drivetrain_goal->set_gear(
      high_gear_ ? frc971::control_loops::drivetrain::Gear::kHighGear
                 : frc971::control_loops::drivetrain::Gear::kLowGear);
  drivetrain_goal->mutable_teleop_command()->set_steering(wheel);
  drivetrain_goal->mutable_teleop_command()->set_throttle(throttle);
  drivetrain_goal->mutable_teleop_command()->set_quick_turn(quickturn);

  // This line can be the only line sending drivetrain goals. If you are
  // using vision, put logic to switch between aligning and driving here.
  generic_robot::QueueManager::GetInstance()
      ->drivetrain_goal_queue()
      ->WriteMessage(drivetrain_goal);
}

}  // namespace citrus_robot
}  // namespace generic_robot
