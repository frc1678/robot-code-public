#include "c2017/citrus_robot/citrus_robot.h"
#include "WPILib.h"
#include "c2017/queue_manager/queue_manager.h"
#include "muan/wpilib/queue_types.h"
#include "muan/utils/threading_utils.h"

namespace c2017 {
namespace citrus_robot {

CitrusRobot::CitrusRobot()
    : throttle_{1, c2017::QueueManager::GetInstance()->throttle_status_queue()},
      wheel_{0, c2017::QueueManager::GetInstance()->wheel_status_queue()},
      gamepad_{2, c2017::QueueManager::GetInstance()->manipulator_status_queue()},
      ds_sender_{c2017::QueueManager::GetInstance()->driver_station_queue()},
      ds_reader_{c2017::QueueManager::GetInstance()->driver_station_queue()->MakeReader()} {

  align_shoot_ = throttle_.MakeButton(1);  // Joystick Trigger
  driver_score_ground_gear_ = throttle_.MakeButton(2);
  quickturn_ = wheel_.MakeButton(5);

  gear_intake_down_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::A_BUTTON));
  operator_score_ground_gear_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::B_BUTTON));
  ball_reverse_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::X_BUTTON));
  just_shoot_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::Y_BUTTON));
  stop_shooting_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::LEFT_BUMPER));
  ball_intake_slow_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::RIGHT_BUMPER));
  agitate_ = gamepad_.MakeAxis(2, .7);
  ball_intake_fast_ = gamepad_.MakeAxis(3, .7);
  climb_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::BACK));
  just_spinup_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::START));
  toggle_distance_align_ =
      gamepad_.MakeButton(uint32_t(muan::teleop::XBox::LEFT_CLICK_IN));
  toggle_magazine_ = gamepad_.MakePov(0, muan::teleop::Pov::kEast);
  drop_balls_ = gamepad_.MakeAxis(1, .7);  // Left Joystick South
}

void CitrusRobot::operator()() {
  aos::time::PhasedLoop phased_loop(std::chrono::milliseconds(20));

  aos::SetCurrentThreadRealtimePriority(10);
  muan::utils::SetCurrentThreadName("CitrusRobot");

  running_ = true;
  while (running_) {
    throttle_.Update();
    wheel_.Update();
    gamepad_.Update();

    // Update joysticks
    // TODO(livyt) put this back in main.cpp and read from queues here.
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
      // lemonscript_.Start();  // Weird to call Start in a loop, but it's a setter so it's fine
      ::frc971::control_loops::drivetrain::GoalProto dt_goal;
      dt_goal->mutable_path_command()->set_x_goal(1.5);
      dt_goal->mutable_path_command()->set_y_goal(-0.0);
      dt_goal->mutable_path_command()->set_theta_goal(2.0);

      dt_goal->mutable_linear_constraints()->set_max_velocity(3.0);
      dt_goal->mutable_linear_constraints()->set_max_acceleration(3.0);
      dt_goal->mutable_angular_constraints()->set_max_velocity(3.0);
      dt_goal->mutable_angular_constraints()->set_max_acceleration(3.0);

      QueueManager::GetInstance()->drivetrain_goal_queue()->WriteMessage(dt_goal);
    } else if (ds_status->mode() == RobotMode::TELEOP) {
      lemonscript_.Stop();  // Weirder to do this, but it works :/

      auto superstructure_status =
          c2017::QueueManager::GetInstance()->superstructure_status_queue()->ReadLastMessage();
      if (superstructure_status) {
        muan::teleop::XBoxRumbleProto xbox_rumble;
        xbox_rumble->set_rumble(superstructure_status.value()->rumble_on() ? 1 : 0);
        c2017::QueueManager::GetInstance()->xbox_rumble_queue()->WriteMessage(xbox_rumble);
      }

      SendDrivetrainMessage();
      SendSuperstructureMessage();
    }
  }

  ds_sender_.Send();
}

void CitrusRobot::SendSuperstructureMessage() {
  if (ball_intake_fast_->is_pressed()) {
    intake_group_goal_->set_ground_ball_rollers(intake_group::GROUND_BALL_IN);
  } else if (ball_intake_slow_->is_pressed()) {
    intake_group_goal_->set_ground_ball_rollers(intake_group::GROUND_BALL_IN_SLOW);
  } else if (ball_reverse_->is_pressed()) {
    intake_group_goal_->set_ground_ball_rollers(intake_group::GROUND_BALL_OUT);
    magazine_goal_->set_lower_goal(magazine::LOWER_BACKWARD);
  } else {
    intake_group_goal_->set_ground_ball_rollers(intake_group::GROUND_BALL_NONE);
  }

  // Toggle the magazine
  magazine_out_ = magazine_out_ != toggle_magazine_->was_clicked();
  intake_group_goal_->set_side_magazine_open(magazine_out_);
  intake_group_goal_->set_front_magazine_open(magazine_out_);

  intake_group_goal_->set_agitate(agitate_->is_pressed());

  if (gear_intake_down_->was_clicked()) {
    intake_group_goal_->set_ground_gear_intake(intake_group::GROUND_GEAR_DROP);
  } else if (gear_intake_down_->was_released()) {
    intake_group_goal_->set_ground_gear_intake(intake_group::GROUND_GEAR_RISE);
  } else if (operator_score_ground_gear_->is_pressed() || driver_score_ground_gear_->is_pressed()) {
    intake_group_goal_->set_ground_gear_intake(intake_group::GROUND_GEAR_SCORE);
  } else {
    intake_group_goal_->set_ground_gear_intake(intake_group::GROUND_GEAR_NONE);
  }

  if (drop_balls_->is_pressed()) {
    intake_group_goal_->set_ground_gear_intake(intake_group::GROUND_GEAR_START_DROPPING_BALLS);
  }

  if (drop_balls_->was_released()) {
    intake_group_goal_->set_ground_gear_intake(intake_group::GROUND_GEAR_STOP_DROPPING_BALLS);
  }

  intake_group_goal_->set_ground_ball_position(ball_intake_down_ ? intake_group::GROUND_BALL_DOWN
                                                                 : intake_group::GROUND_BALL_UP);

  if (climb_->was_clicked()) {
    // Kelly - Gamepad Button
    currently_climbing_ = !currently_climbing_;
  }

  shooter_group_goal_->set_should_climb(currently_climbing_);

  // Shooting buttons
  if (align_shoot_->was_clicked()) {
    // Avery - Throttle Button
    using_vision_ = true;
    if (vision_aligned_) {
      shooter_group_goal_->set_wheel(shooter_group::Wheel::BOTH);
    } else {
      shooter_group_goal_->set_wheel(shooter_group::Wheel::SPINUP);
    }
  } else if (just_spinup_->is_pressed()) {
    // Kelly - Gamepad Button
    shooter_group_goal_->set_wheel(shooter_group::Wheel::SPINUP);
    using_vision_ = false;
  } else if (just_shoot_->is_pressed()) {
    // Kelly - Gamepad Button
    intake_group_goal_->set_ground_ball_position(intake_group::GROUND_BALL_DOWN);
    shooter_group_goal_->set_wheel(shooter_group::Wheel::BOTH);
    using_vision_ = false;
  } else if (stop_shooting_->was_clicked()) {
    // Kelly - Gamepad Button
    shooter_group_goal_->set_wheel(shooter_group::Wheel::IDLE);
    intake_group_goal_->set_ground_ball_rollers(intake_group::GROUND_BALL_NONE);
    intake_group_goal_->set_ground_ball_position(intake_group::GROUND_BALL_UP);
    using_vision_ = false;
  }

  c2017::QueueManager::GetInstance()->intake_group_goal_queue()->WriteMessage(intake_group_goal_);
  c2017::QueueManager::GetInstance()->shooter_group_goal_queue()->WriteMessage(shooter_group_goal_);
}

void CitrusRobot::SendDrivetrainMessage() {
  frc971::control_loops::drivetrain::GoalProto drivetrain_goal;

  if (toggle_distance_align_->was_clicked()) {
    use_distance_align_ = !use_distance_align_;
  }

  double throttle = -throttle_.wpilib_joystick()->GetRawAxis(1);
  double wheel = -wheel_.wpilib_joystick()->GetRawAxis(0);
  bool quickturn = quickturn_->is_pressed();

  c2017::vision::VisionGoalProto vision_goal;
  vision_goal->set_should_align(using_vision_);
  vision_goal->set_use_distance_align(use_distance_align_);

  drivetrain_goal->mutable_teleop_command()->set_steering(wheel);
  drivetrain_goal->mutable_teleop_command()->set_throttle(throttle);
  drivetrain_goal->mutable_teleop_command()->set_quick_turn(quickturn);

  auto vision_status = c2017::QueueManager::GetInstance()->vision_status_queue()->ReadLastMessage();
  if (vision_status) {
    vision_aligned_ = vision_status.value()->aligned();
  } else {
    vision_aligned_ = false;
  }

  if (!using_vision_ || !vision_status || vision_status.value()->aligned()) {
    using_vision_ = false;
    c2017::QueueManager::GetInstance()->drivetrain_goal_queue()->WriteMessage(drivetrain_goal);
  }
  c2017::QueueManager::GetInstance()->vision_goal_queue()->WriteMessage(vision_goal);
}

}  // namespace citrus_robot
}  // namespace c2017
