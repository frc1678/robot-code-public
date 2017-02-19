#include "c2017/subsystems/superstructure/superstructure.h"

namespace c2017 {

namespace superstructure {

// Constants for the shooter
// The first value is revolutions per minute, which is then converted to radians per second
constexpr double kFenderVelocity = 3000 * (M_PI * 2) / 60;
constexpr double kHopperVelocity = 3000 * (M_PI * 2) / 60;

SuperStructure::SuperStructure() {}

void SuperStructure::Update() {
  c2017::magazine::MagazineGoalProto magazine_goal;

  auto maybe_shooter_group_goal = QueueManager::GetInstance().shooter_group_goal_queue().ReadLastMessage();
  auto maybe_shooter_status = QueueManager::GetInstance().shooter_status_queue().ReadLastMessage();

  if (maybe_shooter_group_goal && maybe_shooter_status) {
    // MUST call spinup before you call shoot
    auto shooter_group_goal = maybe_shooter_group_goal.value();
    auto shooter_status = maybe_shooter_status.value();

    if (shooter_group_goal->wheel() == c2017::shooter_group::Wheel::SPINUP ||
        shooter_group_goal->wheel() == c2017::shooter_group::Wheel::BOTH) {
      if (shooter_group_goal->position() == c2017::shooter_group::Position::FENDER) {
        shooter_goal_->set_goal_mode(c2017::shooter::ShotMode::FENDER);
        shooter_goal_->set_goal_velocity(kFenderVelocity);
      } else if (shooter_group_goal->position() == c2017::shooter_group::Position::HOPPER) {
        shooter_goal_->set_goal_mode(c2017::shooter::ShotMode::HOPPER);
        shooter_goal_->set_goal_velocity(kHopperVelocity);
      }
    }

    if (shooter_group_goal->wheel() == c2017::shooter_group::Wheel::SHOOT ||
        shooter_group_goal->wheel() == c2017::shooter_group::Wheel::BOTH) {
      if (shooter_status->at_goal() && shooter_status->currently_running()) {
        magazine_goal->set_side_goal(c2017::magazine::SideGoalState::SIDE_PULL_IN);
        magazine_goal->set_upper_goal(c2017::magazine::UpperGoalState::UPPER_FORWARD);
        magazine_goal->set_lower_goal(c2017::magazine::LowerGoalState::LOWER_FORWARD);
        is_shooting_ = true;
      } else {  // Shooter not at speed
        is_shooting_ = false;
        magazine_goal->set_upper_goal(c2017::magazine::UpperGoalState::UPPER_IDLE);
      }
    }

    if (shooter_group_goal->wheel() == c2017::shooter_group::Wheel::IDLE) {
        magazine_goal->set_upper_goal(c2017::magazine::UpperGoalState::UPPER_IDLE);
        magazine_goal->set_side_goal(c2017::magazine::SideGoalState::SIDE_IDLE);
        magazine_goal->set_lower_goal(c2017::magazine::LowerGoalState::LOWER_IDLE);
        superstructure_status_proto_->set_shooting(false);
        shooter_goal_->set_goal_velocity(0.0);
    }

    superstructure_status_proto_->set_shooting(is_shooting_);

    // Climbing!
    superstructure_status_proto_->set_climbing(shooter_group_goal->should_climb());
    magazine_goal->set_magazine_extended(!shooter_group_goal->should_climb());
    climber_goal_->set_climbing(shooter_group_goal->should_climb());
  }

  // Reading the group goal queues
  auto maybe_intake_group_goal = QueueManager::GetInstance().intake_group_goal_queue().ReadLastMessage();

  c2017::ground_gear_intake::GroundGearIntakeGoalProto ground_gear_intake_goal;
  c2017::ground_ball_intake::GroundBallIntakeGoalProto ground_ball_intake_goal;

  if (maybe_intake_group_goal) {
    auto intake_group_goal = maybe_intake_group_goal.value();
    using c2017::intake_group::GroundGearIntakeGoal;
    switch (intake_group_goal->ground_gear_intake()) {
      case intake_group::GROUND_GEAR_NONE:
        ground_gear_intake_goal->set_goal(c2017::ground_gear_intake::NONE);
        break;
      case intake_group::GROUND_GEAR_DROP:
        ground_gear_intake_goal->set_goal(c2017::ground_gear_intake::DROP);
        break;
      case intake_group::GROUND_GEAR_RISE:
        ground_gear_intake_goal->set_goal(c2017::ground_gear_intake::RISE);
        break;
      case intake_group::GROUND_GEAR_SCORE:
        ground_gear_intake_goal->set_goal(c2017::ground_gear_intake::SCORE);
        break;
    }

    bool allow_ground_intake = ground_gear_intake_.current_state() == ground_gear_intake::IDLE ||
                               ground_gear_intake_.current_state() == ground_gear_intake::CARRYING;

    ground_ball_intake_goal->set_intake_up(!(
        allow_ground_intake && intake_group_goal->ground_ball_position() == intake_group::GROUND_BALL_DOWN));

    switch (intake_group_goal->ground_ball_rollers()) {
      case intake_group::GROUND_BALL_NONE:
        ground_ball_intake_goal->set_run_intake(c2017::ground_ball_intake::IDLE);
        break;
      case intake_group::GROUND_BALL_IN:
        ground_ball_intake_goal->set_run_intake(c2017::ground_ball_intake::INTAKE);
        break;
      case intake_group::GROUND_BALL_OUT:
        ground_ball_intake_goal->set_run_intake(c2017::ground_ball_intake::OUTTAKE);
        break;
    }

    if (intake_group_goal->agitate()) {
      magazine_goal->set_side_goal(c2017::magazine::SideGoalState::SIDE_AGITATE);
      magazine_goal->set_upper_goal(c2017::magazine::UpperGoalState::UPPER_IDLE);
    }

    // HP logic
    switch (intake_group_goal->hp_load_type()) {
      case c2017::intake_group::HpLoadType::HP_LOAD_BALLS:
        // hp load balls: hp goal = balls
        magazine_goal->set_hp_intake_goal(c2017::magazine::HPIntakeGoalState::BALLS);
        superstructure_status_proto_->set_load_hp_balls(true);
        superstructure_status_proto_->set_load_hp_gear(false);
        superstructure_status_proto_->set_load_hp_both(false);
        break;
      case c2017::intake_group::HpLoadType::HP_LOAD_GEAR:
        // hp load gear: hp goal = gear
        magazine_goal->set_hp_intake_goal(c2017::magazine::HPIntakeGoalState::GEAR);
        superstructure_status_proto_->set_load_hp_balls(false);
        superstructure_status_proto_->set_load_hp_gear(true);
        superstructure_status_proto_->set_load_hp_both(false);
        break;
      case c2017::intake_group::HpLoadType::HP_LOAD_NONE:
        // hp load none: hp goal = none
        magazine_goal->set_hp_intake_goal(c2017::magazine::HPIntakeGoalState::NONE);
        superstructure_status_proto_->set_load_hp_balls(false);
        superstructure_status_proto_->set_load_hp_gear(false);
        superstructure_status_proto_->set_load_hp_both(false);
        break;
      case c2017::intake_group::HpLoadType::HP_LOAD_BOTH:
        // hp load both: hp goal = both
        magazine_goal->set_hp_intake_goal(c2017::magazine::HPIntakeGoalState::BOTH);
        superstructure_status_proto_->set_load_hp_balls(false);
        superstructure_status_proto_->set_load_hp_gear(false);
        superstructure_status_proto_->set_load_hp_both(true);
        break;
    }

    // This doesn't need to be in any specific it's just passed through to the magazine
    if (intake_group_goal->score_hp_gear()) {
      magazine_goal->set_score_gear(true);
      ground_ball_intake_goal->set_intake_up(false);
    } else {
      magazine_goal->set_score_gear(false);
    }
  }

  ground_gear_intake_.SetGoal(ground_gear_intake_goal);
  ground_ball_intake_.set_goal(ground_ball_intake_goal);

  magazine_.SetGoal(magazine_goal);
  shooter_.SetGoal(shooter_goal_);

  SetWpilibOutput();
  QueueManager::GetInstance().superstructure_status_queue().WriteMessage(superstructure_status_proto_);
}

void SuperStructure::SetWpilibOutput() {
  wpilib::WpilibOutputProto wpilib_output;
  const auto ground_gear_input = QueueManager::GetInstance().ground_gear_input_queue().ReadLastMessage();
  const auto magazine_input = QueueManager::GetInstance().magazine_input_queue().ReadLastMessage();
  const auto shooter_input = QueueManager::GetInstance().shooter_input_queue().ReadLastMessage();
  const auto climber_input = QueueManager::GetInstance().climber_input_queue().ReadLastMessage();
  const auto climber_status = QueueManager::GetInstance().climber_status_queue().ReadLastMessage();
  const auto driver_station = QueueManager::GetInstance().driver_station_queue()->ReadLastMessage();

  bool enable_outputs = true;
  if (driver_station) {
    auto robot_state = driver_station.value();
    enable_outputs = !(robot_state->mode() == RobotMode::DISABLED ||
                       robot_state->mode() == RobotMode::ESTOP || robot_state->brownout());
  }

  if (ground_gear_input) {
    auto ground_gear_intake_output = ground_gear_intake_.Update(ground_gear_input.value(), enable_outputs);
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
      auto shooter_output = shooter_.Update(shooter_input.value(), enable_outputs);
      wpilib_output->set_shooter_hood_up(shooter_output->hood_solenoid());
      wpilib_output->set_shooter_voltage(shooter_output->voltage());
    }
  }

  if (magazine_input) {
    auto magazine_output = magazine_.Update(magazine_input.value(), enable_outputs);
    auto ground_ball_intake_output = ground_ball_intake_.Update(enable_outputs);
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

  QueueManager::GetInstance().superstructure_output_queue().WriteMessage(wpilib_output);
}

}  // namespace superstructure

}  // namespace c2017
