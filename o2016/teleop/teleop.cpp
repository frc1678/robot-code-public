#include "o2016/teleop/teleop.h"
#include "o2016/queue_manager/queue_manager.h"

namespace o2016 {

namespace teleop {

Teleop::Teleop() : throttle_{1}, wheel_{0}, gamepad_{2} {
  shifting_low_ = throttle_.MakeButton(4);
  shifting_high_ = throttle_.MakeButton(5);
  quickturn_ = throttle_.MakeButton(5);
}

void Teleop::Update() {
  // Update joysticks
  throttle_.Update();
  wheel_.Update();
  gamepad_.Update();

  {
    // Drivetrain controls
    o2016::drivetrain::StackDrivetrainGoal drivetrain_goal;

    double throttle = throttle_.wpilib_joystick()->GetRawAxis(1);
    double wheel = wheel_.wpilib_joystick()->GetRawAxis(0);
    bool quickturn = quickturn_->is_pressed();

    double angular;
    double forward = throttle;
    if (quickturn) {
      double angular = wheel;
    } else {
      double angular = throttle * wheel;
    }

    if (shifting_high_->was_clicked()) {
      high_gear_ = true;
    } else if (shifting_low_->was_clicked()) {
      high_gear_ = false;
    }

    drivetrain_goal->mutable_velocity_command()->set_forward_velocity(forward);
    drivetrain_goal->mutable_velocity_command()->set_angular_velocity(angular);
    drivetrain_goal->set_gear(high_gear_ ? o2016::drivetrain::Gear::kHighGear
                                         : o2016::drivetrain::Gear::kLowGear);

    o2016::QueueManager::GetInstance().drivetrain_goal_queue().WriteMessage(
        drivetrain_goal);
  }
}

}  // teleop

}  // o2016
