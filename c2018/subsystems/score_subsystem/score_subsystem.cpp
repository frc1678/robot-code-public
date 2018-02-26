#include "c2018/subsystems/score_subsystem/score_subsystem.h"

namespace c2018 {
namespace score_subsystem {

using muan::queues::QueueManager;
using muan::wpilib::DriverStationProto;

ScoreSubsystem::ScoreSubsystem()
    : goal_reader_{QueueManager<ScoreSubsystemGoalProto>::Fetch()
                       ->MakeReader()},
      input_reader_{
          QueueManager<ScoreSubsystemInputProto>::Fetch()->MakeReader()},
      status_queue_{QueueManager<ScoreSubsystemStatusProto>::Fetch()},
      output_queue_{QueueManager<ScoreSubsystemOutputProto>::Fetch()},
      ds_status_reader_{
          QueueManager<DriverStationProto>::Fetch()->MakeReader()} {}

void ScoreSubsystem::BoundGoal(double* elevator_goal,
                               double* wrist_goal) const {
  if (status_->wrist_angle() > kWristSafeAngle) {
    *elevator_goal = muan::utils::Cap(*elevator_goal, kElevatorWristSafeHeight,
                                      elevator::kElevatorMaxHeight);
  }

  if (status_->elevator_actual_height() < kElevatorWristSafeHeight) {
    *wrist_goal = muan::utils::Cap(*wrist_goal, 0, kWristSafeAngle);
  }
}

void ScoreSubsystem::Update() {
  ScoreSubsystemGoalProto goal;
  ScoreSubsystemInputProto input;
  ScoreSubsystemOutputProto output;
  DriverStationProto driver_station;

  if (!input_reader_.ReadLastMessage(&input)) {
    // TODO(Kyle) handle this gracefully
    return;
  }

  if (!ds_status_reader_.ReadLastMessage(&driver_station)) {
    driver_station->set_battery_voltage(12.0);
  }

  if (!goal_reader_.ReadLastMessage(&goal)) {
    goal->set_score_goal(SCORE_NONE);
    goal->set_intake_goal(INTAKE_NONE);
  }

  RunStateMachine();

  SetGoal(goal);

  double constrained_elevator_height = elevator_height_;
  double constrained_wrist_angle = wrist_angle_;

  BoundGoal(&constrained_elevator_height, &constrained_wrist_angle);

  elevator_.SetGoal(constrained_elevator_height);
  elevator_.Update(input, &output, &status_, driver_station->is_sys_active());

  wrist::IntakeMode intake_mode = wrist::IntakeMode::IDLE;
  switch (state_) {
    case ScoreSubsystemState::CALIBRATING:
    case ScoreSubsystemState::HOLDING:
      intake_mode = wrist::IntakeMode::IDLE;
      break;
    case ScoreSubsystemState::INTAKING:
      intake_mode = wrist::IntakeMode::IN;
      break;
    case ScoreSubsystemState::INTAKING_ONLY:
      intake_mode = wrist::IntakeMode::IN;
      break;
    case ScoreSubsystemState::SCORING_SLOW:
      intake_mode = wrist::IntakeMode::OUT_SLOW;
      break;
    case ScoreSubsystemState::SCORING_FAST:
      intake_mode = wrist::IntakeMode::OUT_FAST;
      break;
  }

  wrist_.SetGoal(constrained_wrist_angle, intake_mode);
  wrist_.Update(input, &output, &status_, driver_station->is_sys_active());

  status_->set_state(state_);

  output_queue_->WriteMessage(output);
  status_queue_->WriteMessage(status_);
}

void ScoreSubsystem::SetGoal(const ScoreSubsystemGoalProto& goal) {
  switch (goal->score_goal()) {
    case SCORE_NONE:
      break;
    case INTAKE_0:
      elevator_height_ = kElevatorIntake0;
      wrist_angle_ = kWristForwardAngle;
      break;
    case INTAKE_1:
      elevator_height_ = kElevatorIntake1;
      wrist_angle_ = kWristForwardAngle;
      break;
    case INTAKE_2:
      elevator_height_ = kElevatorIntake2;
      wrist_angle_ = kWristForwardAngle;
      break;
    case STOW:
      elevator_height_ = kElevatorStow;
      wrist_angle_ = kWristStowAngle;
      break;
    case SWITCH:
      elevator_height_ = kElevatorSwitch;
      wrist_angle_ = kWristForwardAngle;
      break;
    case SCALE_LOW_FORWARD:
      elevator_height_ = kElevatorBaseHeight;
      wrist_angle_ = kWristForwardAngle;
      break;
    case SCALE_LOW_REVERSE:
      elevator_height_ = kElevatorBaseHeight + kElevatorReversedOffset;
      wrist_angle_ = kWristBackwardAngle;
      break;
    case SCALE_MID_FORWARD:
      elevator_height_ = kElevatorBaseHeight + kCubeHeight;
      wrist_angle_ = kWristForwardAngle;
      break;
    case SCALE_MID_REVERSE:
      elevator_height_ = kElevatorBaseHeight + kCubeHeight + kElevatorReversedOffset;
      wrist_angle_ = kWristBackwardAngle;
      break;
    case SCALE_HIGH_FORWARD:
      elevator_height_ = kElevatorBaseHeight + 2 * kCubeHeight;
      wrist_angle_ = kWristForwardAngle;
      break;
    case SCALE_HIGH_REVERSE:
      elevator_height_ = kElevatorBaseHeight + 2 * kCubeHeight + kElevatorReversedOffset;
      wrist_angle_ = kWristBackwardAngle;
      break;
    case SCALE_SUPER_HIGH_FORWARD:
      elevator_height_ = elevator::kElevatorMaxHeight - 0.02;
      wrist_angle_ = kWristTiltUpAngle;
      break;
    case SCALE_SUPER_HIGH_REVERSE:
      elevator_height_ = kElevatorBaseHeight + 3 * kCubeHeight + kElevatorReversedOffset;
      wrist_angle_ = kWristBackwardAngle;
      break;
    case SCALE_SHOOT:
      elevator_height_ = 1.0;
      wrist_angle_ = kWristShootAngle;
      break;
    case EXCHANGE:
      elevator_height_ = kElevatorExchange;
      wrist_angle_ = kWristForwardAngle;
      break;
    case PORTAL:
      elevator_height_ = kElevatorPortal;
      wrist_angle_ = kWristPortalAngle;
      break;
  }

  switch (goal->intake_goal()) {
    case INTAKE_NONE:
      // Just let the state machine take over
      break;
    case INTAKE_ONLY:
      GoToState(ScoreSubsystemState::INTAKING_ONLY);
      break;
    case INTAKE:
      GoToState(ScoreSubsystemState::INTAKING);
      break;
    case OUTTAKE_SLOW:
      GoToState(ScoreSubsystemState::SCORING_SLOW);
      break;
    case OUTTAKE_FAST:
      GoToState(ScoreSubsystemState::SCORING_FAST);
      break;
    case FORCE_STOP:
      GoToState(ScoreSubsystemState::HOLDING);
      break;
  }
}

void ScoreSubsystem::RunStateMachine() {
  switch (state_) {
    case ScoreSubsystemState::CALIBRATING:
      elevator_height_ = status_->elevator_actual_height();
      wrist_angle_ = status_->wrist_angle();
      if (status_->wrist_calibrated() && status_->elevator_calibrated()) {
        // These need to be set right away because calibration moves the
        // goalposts.
        elevator_height_ = kElevatorStow;
        wrist_angle_ = kWristStowAngle;

        GoToState(HOLDING);
      }
      break;
    case HOLDING:
      break;
    case INTAKING:
      if (status_->has_cube()) {
        elevator_height_ = kElevatorStow;
        wrist_angle_ = kWristStowAngle;
        GoToState(HOLDING);
      }
      break;
    case INTAKING_ONLY:
      break;
    case SCORING_FAST:
    case SCORING_SLOW:
      break;
  }
}

void ScoreSubsystem::GoToState(ScoreSubsystemState desired_state) {
  switch (state_) {
    case ScoreSubsystemState::CALIBRATING:
      if (wrist_.is_calibrated() && elevator_.is_calibrated()) {
        state_ = desired_state;
      } else {
        LOG(ERROR, "Tried to go to invalid state %d while calibrating!",
              static_cast<int>(desired_state));
      }
      break;
    case ScoreSubsystemState::HOLDING:
    case ScoreSubsystemState::INTAKING:
    case ScoreSubsystemState::INTAKING_ONLY:
    case ScoreSubsystemState::SCORING_FAST:
    case ScoreSubsystemState::SCORING_SLOW:
      state_ = desired_state;
      break;
  }
}

}  // namespace score_subsystem
}  // namespace c2018
