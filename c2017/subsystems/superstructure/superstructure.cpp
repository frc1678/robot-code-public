#include "c2017/subsystems/superstructure/superstructure.h"

namespace c2017 {

namespace superstructure {

SuperStructure::SuperStructure() {}

void SuperStructure::Update() {
  c2017::magazine::MagazineGoalProto magazine_goal;
  c2017::shooter::ShooterGoalProto shooter_goal;
  c2017::climber::ClimberGoalProto climber_goal;
  c2017::ground_gear_intake::GroundGearIntakeGoalProto ground_gear_intake_goal;
  c2017::ground_ball_intake::GroundBallIntakeGoalProto ground_ball_intake_goal;

  c2017::superstructure::SuperstructureStatusProto superstructure_status;

  const auto maybe_climber_goal = QueueManager::GetInstance().climber_goal_queue().ReadLastMessage();
  const auto maybe_shooter_group_goal =
      QueueManager::GetInstance().shooter_group_goal_queue().ReadLastMessage();
  const auto maybe_shooter_status = QueueManager::GetInstance().shooter_status_queue().ReadLastMessage();
  const auto maybe_ground_gear_intake_status =
      QueueManager::GetInstance().ground_gear_status_queue().ReadLastMessage();
  const auto maybe_intake_group_goal =
      QueueManager::GetInstance().intake_group_goal_queue().ReadLastMessage();

  const auto maybe_shooter_input = QueueManager::GetInstance().shooter_input_queue().ReadLastMessage();
  const auto maybe_ground_gear_input =
      QueueManager::GetInstance().ground_gear_input_queue().ReadLastMessage();
  const auto maybe_climber_input = QueueManager::GetInstance().climber_input_queue().ReadLastMessage();
  const auto maybe_driver_station = QueueManager::GetInstance().driver_station_queue()->ReadLastMessage();

  // Goal interpretation
  if (maybe_shooter_group_goal && maybe_shooter_status) {
    const auto shooter_group_goal = maybe_shooter_group_goal.value();
    const auto shooter_status = maybe_shooter_status.value();

    switch (shooter_group_goal->wheel()) {
      case shooter_group::IDLE:
        shooter_state_ = SuperstructureStatus::kShooterIdle;
        break;
      case shooter_group::SPINUP:
        shooter_state_ = SuperstructureStatus::kShooterSpinup;
        break;
      case shooter_group::SHOOT:
        shooter_state_ = SuperstructureStatus::kShooterShooting;
        break;
      case shooter_group::BOTH:
        if (shooter_status->currently_running() && shooter_status->at_goal()) {
          shooter_state_ = SuperstructureStatus::kShooterShooting;
        } else {
          shooter_state_ = SuperstructureStatus::kShooterSpinup;
        }
        break;
    }
  }

  bool is_climbing = false;
  if (maybe_climber_goal) {
    climber_goal = maybe_climber_goal.value();
    if (climber_goal->climbing()) {
      is_climbing = true;
      shooter_state_ = SuperstructureStatus::kShooterIdle;
      superstructure_status->set_climbing(true);
    }
  }

  if (maybe_intake_group_goal) {
    const auto intake_group_goal = maybe_intake_group_goal.value();
    switch (intake_group_goal->ground_gear_intake()) {
      case intake_group::GROUND_GEAR_NONE:
        ground_gear_intake_goal->set_goal(ground_gear_intake::NONE);
        break;
      case intake_group::GROUND_GEAR_DROP:
        ground_gear_intake_goal->set_goal(ground_gear_intake::DROP);
        break;
      case intake_group::GROUND_GEAR_RISE:
        ground_gear_intake_goal->set_goal(ground_gear_intake::RISE);
        break;
      case intake_group::GROUND_GEAR_SCORE:
        ground_gear_intake_goal->set_goal(ground_gear_intake::SCORE);
        break;
    }

    ground_ball_intake_goal->set_intake_up(intake_group_goal->ground_ball_position() ==
                                           intake_group::GROUND_BALL_UP);

    switch (intake_group_goal->ground_ball_rollers()) {
      case intake_group::GROUND_BALL_NONE:
        ground_ball_intake_goal->set_run_intake(ground_ball_intake::IDLE);
        break;
      case intake_group::GROUND_BALL_IN:
        ground_ball_intake_goal->set_run_intake(ground_ball_intake::INTAKE);
        break;
      case intake_group::GROUND_BALL_OUT:
        ground_ball_intake_goal->set_run_intake(ground_ball_intake::OUTTAKE);
        break;
    }

    if (intake_group_goal->agitate()) {
      magazine_goal->set_side_goal(c2017::magazine::SideGoalState::SIDE_AGITATE);
      magazine_goal->set_upper_goal(c2017::magazine::UpperGoalState::UPPER_IDLE);
    }
    magazine_goal->set_magazine_extended(intake_group_goal->magazine_open());
  }

  if (shooter_state_ == SuperstructureStatus::kShooterShooting) {
    magazine_goal->set_upper_goal(magazine::UPPER_FORWARD);
    magazine_goal->set_lower_goal(magazine::LOWER_FORWARD);
    magazine_goal->set_side_goal(magazine::SIDE_PULL_IN);
    ground_ball_intake_goal->set_intake_up(false);
  }

  if (is_climbing) {
    ground_gear_intake_goal->set_goal(c2017::ground_gear_intake::OUTTAKE);
  }

  shooter_goal->set_goal_velocity(shooter_state_ == SuperstructureStatus::kShooterIdle ? 0.0
                                                                                       : kShooterVelocity);

  if (ground_gear_intake_goal->goal() == ground_gear_intake::DROP ||
      ground_gear_intake_goal->goal() == ground_gear_intake::SCORE) {
    ground_ball_intake_goal->set_intake_up(true);
  }

  if (maybe_ground_gear_intake_status) {
    const auto ground_gear_intake_status = maybe_ground_gear_intake_status.value();
    if (ground_gear_intake_status->current_state() == ground_gear_intake::INTAKING ||
        ground_gear_intake_status->current_state() == ground_gear_intake::PICKING_UP ||
        ground_gear_intake_status->current_state() == ground_gear_intake::SCORING) {
      ground_ball_intake_goal->set_intake_up(true);
    }
  }

  ground_gear_intake_.SetGoal(ground_gear_intake_goal);
  ground_ball_intake_.set_goal(ground_ball_intake_goal);

  magazine_.SetGoal(magazine_goal);
  shooter_.SetGoal(shooter_goal);
  climber_.SetGoal(climber_goal);

  superstructure_status->set_state(shooter_state_);
  QueueManager::GetInstance().superstructure_status_queue().WriteMessage(superstructure_status);

  bool outputs_enabled = false;

  if (maybe_driver_station) {
    const auto driver_station = maybe_driver_station.value();
    outputs_enabled = driver_station->is_sys_active();
  }

  // Update the mechanisms
  shooter::ShooterOutputProto shooter_output;
  if (maybe_shooter_input) {
    shooter_output = shooter_.Update(maybe_shooter_input.value(), outputs_enabled);
  }

  ground_gear_intake::GroundGearIntakeOutputProto ground_gear_output;
  if (maybe_ground_gear_input) {
    ground_gear_output = ground_gear_intake_.Update(maybe_ground_gear_input.value(), outputs_enabled);
  }

  auto ground_ball_output = ground_ball_intake_.Update(outputs_enabled);
  auto magazine_output = magazine_.Update(outputs_enabled);

  climber::ClimberOutputProto climber_output;
  if (maybe_climber_input) {
    climber_output = climber_.Update(maybe_climber_input.value(), outputs_enabled);
  }

  wpilib::WpilibOutputProto output;

  // Have the magazine output for the roller take precedence over the intake, as shooting is more important
  // than outtaking. This will be unnecessary when they're run off of separate motors.
  if (magazine_output->lower_voltage() != 0.0) {
    output->set_main_roller_voltage(magazine_output->lower_voltage());
  } else {
    output->set_main_roller_voltage(ground_ball_output->roller_voltage());
  }

  output->set_ball_intake_down(!ground_ball_output->intake_up());
  output->set_ground_gear_voltage(ground_gear_output->roller_voltage());
  output->set_ground_gear_down(ground_gear_output->intake_down());
  output->set_upper_conveyor_voltage(magazine_output->upper_voltage());
  output->set_side_conveyor_voltage(magazine_output->side_voltage());
  output->set_magazine_open(magazine_output->magazine_extended());
  output->set_shooter_voltage(shooter_output->shooter_voltage());
  output->set_accelerator_voltage(is_climbing ? climber_output->voltage()
                                              : shooter_output->accelerator_voltage());
  output->set_climber_engaged(is_climbing);

  QueueManager::GetInstance().superstructure_output_queue().WriteMessage(output);
}

}  // namespace superstructure

}  // namespace c2017
