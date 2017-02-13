#include "c2017/subsystems/superstructure/superstructure.h"

namespace c2017 {

namespace superstructure {

// Constants for the shooter
// The first value is revolutions per minute, which is then converted to radians per second
constexpr double kFenderVelocity = 3000 * (M_PI * 2) / 60;
constexpr double kHopperVelocity = 3000 * (M_PI * 2) / 60;

SuperStructure::SuperStructure() {}

void SuperStructure::Update() {
  UpdateShooter();
  UpdateIntake();

  ground_gear_intake_.SetGoal(ground_gear_intake_goal_);
  magazine_.SetGoal(magazine_goal_);
  ground_ball_intake_.set_goal(ground_ball_intake_goal_);
  shooter_.SetGoal(shooter_goal_);

  SetWpilibOutput();
  QueueManager::GetInstance().superstructure_status_queue().WriteMessage(superstructure_status_proto_);
}

void SuperStructure::UpdateShooter() {
  auto shooter_group_goal = QueueManager::GetInstance().shooter_group_goal_queue().ReadLastMessage();
  auto shooter_status = QueueManager::GetInstance().shooter_status_queue().ReadLastMessage();

  if (shooter_group_goal && shooter_status) {
    // MUST call spinup before you call shoot
    switch (shooter_group_goal.value()->wheel()) {
      case c2017::shooter_group::Wheel::BOTH:
        Spinup(shooter_group_goal.value());
        Shoot(shooter_status.value());
        break;
      case c2017::shooter_group::Wheel::SHOOT:
        Shoot(shooter_status.value());
        break;
      case c2017::shooter_group::Wheel::SPINUP:
        Spinup(shooter_group_goal.value());
        break;
      case c2017::shooter_group::Wheel::IDLE:
        magazine_goal_->set_upper_goal(c2017::magazine::UpperGoalState::UPPER_IDLE);
        magazine_goal_->set_side_goal(c2017::magazine::SideGoalState::SIDE_IDLE);
        magazine_goal_->set_lower_goal(c2017::magazine::LowerGoalState::LOWER_IDLE);
        superstructure_status_proto_->set_shooting(false);
        shooter_goal_->set_goal_velocity(0.0);
        break;
    }

    superstructure_status_proto_->set_shooting(is_shooting_);

    // Climbing!
    superstructure_status_proto_->set_climbing(shooter_group_goal.value()->should_climb());
    magazine_goal_->set_magazine_extended(!shooter_group_goal.value()->should_climb());
    climber_goal_->set_climbing(shooter_group_goal.value()->should_climb());
  }
}

void SuperStructure::Shoot(const c2017::shooter::ShooterStatusProto& shooter_status) {
  if (shooter_status->at_goal() && shooter_status->currently_running()) {
    magazine_goal_->set_side_goal(c2017::magazine::SideGoalState::SIDE_PULL_IN);
    magazine_goal_->set_upper_goal(c2017::magazine::UpperGoalState::UPPER_FORWARD);
    magazine_goal_->set_lower_goal(c2017::magazine::LowerGoalState::LOWER_FORWARD);
    is_shooting_ = true;
  } else {  // Shooter not at speed
    is_shooting_ = false;
    magazine_goal_->set_upper_goal(c2017::magazine::UpperGoalState::UPPER_IDLE);
  }
}

void SuperStructure::Spinup(const c2017::shooter_group::ShooterGroupGoalProto& shooter_group_goal) {
  if (shooter_group_goal->position() == c2017::shooter_group::Position::FENDER) {
    shooter_goal_->set_goal_mode(c2017::shooter::ShotMode::FENDER);
    shooter_goal_->set_goal_velocity(kFenderVelocity);
  } else if (shooter_group_goal->position() == c2017::shooter_group::Position::HOPPER) {
    shooter_goal_->set_goal_mode(c2017::shooter::ShotMode::HOPPER);
    shooter_goal_->set_goal_velocity(kHopperVelocity);
  }
}

void SuperStructure::UpdateIntake() {
  // Reading the group goal queues
  auto intake_group_goal = QueueManager::GetInstance().intake_group_goal_queue().ReadLastMessage();

  // GOAL SETTING
  if (intake_group_goal) {
    switch (intake_group_goal.value()->ground_intake_position()) {
      case c2017::intake_group::GroundIntakePosition::INTAKE_BALLS:
        // Intake balls: ball intake down
        ground_ball_intake_goal_->set_intake_up(false);
        superstructure_status_proto_->set_gear_intaking(false);
        superstructure_status_proto_->set_ball_reverse(false);
        superstructure_status_proto_->set_ball_intaking(true);
        // Gear idle: carry gear
        ground_gear_intake_goal_->set_goal(ground_gear_intake::CARRY);
        break;
      case c2017::intake_group::GroundIntakePosition::INTAKE_GEAR:
        // Ground intake gear: ball intake up
        ground_ball_intake_goal_->set_intake_up(true);
        superstructure_status_proto_->set_ball_intaking(false);
        superstructure_status_proto_->set_gear_intaking(true);
        superstructure_status_proto_->set_ball_reverse(false);

        // Gear intake rollers: pickup gear
        ground_gear_intake_goal_->set_goal(ground_gear_intake::PICKUP);
        break;
      case c2017::intake_group::GroundIntakePosition::INTAKE_NONE:
        // Intake none: ball intake up, carry gear
        ground_ball_intake_goal_->set_intake_up(true);
        ground_gear_intake_goal_->set_goal(ground_gear_intake::CARRY);
        superstructure_status_proto_->set_gear_intaking(false);
        superstructure_status_proto_->set_ball_intaking(false);
        superstructure_status_proto_->set_ball_reverse(false);
        break;
    }

    switch (intake_group_goal.value()->gear_intake()) {
      case c2017::intake_group::GearIntakeRollers::GEAR_OUTTAKE:
        // Gear rollers outtake: score gear, ball intake up
        ground_gear_intake_goal_->set_goal(ground_gear_intake::SCORE);
        superstructure_status_proto_->set_gear_scoring(true);
        break;
      case c2017::intake_group::GearIntakeRollers::GEAR_INTAKE:
        // Gear rollers intake: intake gear, ball intake up
        ground_gear_intake_goal_->set_goal(ground_gear_intake::PICKUP);
        superstructure_status_proto_->set_gear_intaking(true);
        superstructure_status_proto_->set_gear_scoring(false);
        break;
      case c2017::intake_group::GearIntakeRollers::GEAR_IDLE:
        // Gear rollers idle: idle gear
        ground_gear_intake_goal_->set_goal(ground_gear_intake::IDLE);
        superstructure_status_proto_->set_gear_scoring(false);
        break;
    }

    // Ball intake rollers
    switch (intake_group_goal.value()->roller()) {
      case c2017::intake_group::BallIntakeRollers::ROLLERS_INTAKE:
        // Rollers intake
        ground_ball_intake_goal_->set_run_intake(c2017::ground_ball_intake::RollerGoal::INTAKE);
        magazine_goal_->set_upper_goal(c2017::magazine::UpperGoalState::UPPER_BACKWARD);
        superstructure_status_proto_->set_ball_intaking(true);
        superstructure_status_proto_->set_ball_reverse(false);
        break;
      case c2017::intake_group::BallIntakeRollers::ROLLERS_OUTTAKE:
        // Rollers outtake
        superstructure_status_proto_->set_ball_intaking(false);
        superstructure_status_proto_->set_ball_reverse(true);
        ground_ball_intake_goal_->set_run_intake(c2017::ground_ball_intake::RollerGoal::OUTTAKE);
        break;
      case c2017::intake_group::BallIntakeRollers::ROLLERS_AGITATE:
        magazine_goal_->set_side_goal(c2017::magazine::SideGoalState::SIDE_AGITATE);
        magazine_goal_->set_upper_goal(c2017::magazine::UpperGoalState::UPPER_IDLE);
        break;
      case c2017::intake_group::BallIntakeRollers::ROLLERS_IDLE:
        if (!is_shooting_) {
          // We don't want to read if the shooter is shooting from the status because that isn't accurate, so
          // we have a member variable to track this
          magazine_goal_->set_side_goal(c2017::magazine::SideGoalState::SIDE_IDLE);
          magazine_goal_->set_upper_goal(c2017::magazine::UpperGoalState::UPPER_IDLE);
          ground_ball_intake_goal_->set_run_intake(c2017::ground_ball_intake::RollerGoal::IDLE);
          superstructure_status_proto_->set_ball_intaking(false);
          superstructure_status_proto_->set_ball_reverse(false);
        }
        break;
    }
    // HP logic
    switch (intake_group_goal.value()->hp_load_type()) {
      case c2017::intake_group::HpLoadType::HP_LOAD_BALLS:
        // hp load balls: hp goal = balls
        magazine_goal_->set_hp_intake_goal(c2017::magazine::HPIntakeGoalState::BALLS);
        superstructure_status_proto_->set_load_hp_balls(true);
        superstructure_status_proto_->set_load_hp_gear(false);
        superstructure_status_proto_->set_load_hp_both(false);
        break;
      case c2017::intake_group::HpLoadType::HP_LOAD_GEAR:
        // hp load gear: hp goal = gear
        magazine_goal_->set_hp_intake_goal(c2017::magazine::HPIntakeGoalState::GEAR);
        superstructure_status_proto_->set_load_hp_balls(false);
        superstructure_status_proto_->set_load_hp_gear(true);
        superstructure_status_proto_->set_load_hp_both(false);
        break;
      case c2017::intake_group::HpLoadType::HP_LOAD_NONE:
        // hp load none: hp goal = none
        magazine_goal_->set_hp_intake_goal(c2017::magazine::HPIntakeGoalState::NONE);
        superstructure_status_proto_->set_load_hp_balls(false);
        superstructure_status_proto_->set_load_hp_gear(false);
        superstructure_status_proto_->set_load_hp_both(false);
        break;
      case c2017::intake_group::HpLoadType::HP_LOAD_BOTH:
        // hp load both: hp goal = both
        magazine_goal_->set_hp_intake_goal(c2017::magazine::HPIntakeGoalState::BOTH);
        superstructure_status_proto_->set_load_hp_balls(false);
        superstructure_status_proto_->set_load_hp_gear(false);
        superstructure_status_proto_->set_load_hp_both(true);
        break;
    }

    // This doesn't need to be in any specific it's just passed through to the magazine
    magazine_goal_->set_score_gear(intake_group_goal.value()->score_hp_gear());
  }
}

void SuperStructure::SetWpilibOutput() {
  wpilib::WpilibOutputProto wpilib_output;
  const auto ground_gear_input =
      QueueManager::GetInstance().ground_gear_input_queue().ReadLastMessage();
  const auto magazine_input =
      QueueManager::GetInstance().magazine_input_queue().ReadLastMessage();
  const auto shooter_input =
      QueueManager::GetInstance().shooter_input_queue().ReadLastMessage();
  const auto climber_input =
      QueueManager::GetInstance().climber_input_queue().ReadLastMessage();
  const auto climber_status =
      QueueManager::GetInstance().climber_status_queue().ReadLastMessage();
  const auto driver_station =
      QueueManager::GetInstance().driver_station_queue()->ReadLastMessage();

  if (driver_station) {
    auto robot_state = driver_station.value();
    bool enable_outputs = !(robot_state->mode() == RobotMode::DISABLED ||
                            robot_state->mode() == RobotMode::ESTOP ||
                            robot_state->brownout());

    if (ground_gear_input) {
      auto ground_gear_intake_output =
          ground_gear_intake_.Update(ground_gear_input.value(), driver_station.value());
      wpilib_output->set_ground_gear_down(ground_gear_intake_output->intake_down());
      wpilib_output->set_ground_gear_voltage(ground_gear_intake_output->roller_voltage());
    }

    if (climber_goal_->climbing()) {
      if (climber_input) {
        auto climber_output = climber_.Update(climber_input.value(), enable_outputs);
        wpilib_output->set_shooter_voltage(climber_output->voltage());
      }
    } else {
      if (shooter_input) {
        auto shooter_output = shooter_.Update(shooter_input.value(), driver_station.value());
        wpilib_output->set_shooter_hood_up(shooter_output->hood_solenoid());
        wpilib_output->set_shooter_voltage(shooter_output->voltage());
      }
    }

    if (magazine_input) {
      auto magazine_output = magazine_.Update(magazine_input.value(), driver_station.value());
      auto ground_ball_intake_output = ground_ball_intake_.Update(driver_station.value());
      wpilib_output->set_gear_shutter_open(magazine_output->gear_shutter_open());
      wpilib_output->set_upper_conveyor_voltage(magazine_output->upper_voltage());
      wpilib_output->set_side_conveyor_voltage(magazine_output->side_voltage());
      wpilib_output->set_hp_gear_open(!magazine_output->gear_intake_closed());
      wpilib_output->set_magazine_open(magazine_output->magazine_extended());
      wpilib_output->set_ball_intake_down(!ground_ball_intake_output->intake_up());
      if (is_shooting_) {
        // If shooting, we want to have the magazine run the lower conveyor
        wpilib_output->set_main_roller_voltage(magazine_output->lower_voltage());
      } else {
        // If not shooting, the ball intake should control the lower conveyor
        wpilib_output->set_main_roller_voltage(ground_ball_intake_output->roller_voltage());
      }
    }
  }

  QueueManager::GetInstance().superstructure_output_queue().WriteMessage(wpilib_output);
}

}  // namespace superstructure

}  // namespace c2017
