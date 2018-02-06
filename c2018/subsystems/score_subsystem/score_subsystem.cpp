#include "c2018/subsystems/score_subsystem/score_subsystem.h"

namespace c2018 {
namespace score_subsystem {

using muan::queues::QueueManager;
using muan::wpilib::DriverStationProto;

ScoreSubsystem::ScoreSubsystem()
    : goal_reader_{QueueManager<ScoreSubsystemGoalProto>::Fetch()
                       ->MakeReader()},
      status_queue_{QueueManager<ScoreSubsystemStatusProto>::Fetch()},
      input_reader_{
          QueueManager<ScoreSubsystemInputProto>::Fetch()->MakeReader()},
      output_queue_{QueueManager<ScoreSubsystemOutputProto>::Fetch()},
      ds_status_reader_{
          QueueManager<DriverStationProto>::Fetch()->MakeReader()} {}

void ScoreSubsystem::Update() {
  ScoreSubsystemGoalProto goal;
  ScoreSubsystemInputProto input;
  ScoreSubsystemOutputProto output;
  DriverStationProto driver_station;

  if (!input_reader_.ReadLastMessage(&input)) {
    return;
  }

  if (!ds_status_reader_.ReadLastMessage(&driver_station)) {
    driver_station->set_battery_voltage(12.0);
  }

  if (!goal_reader_.ReadLastMessage(&goal)) {
    // Set default goal
    wrist_angle_ = 0;
    intake_mode_ = IDLE;
    // Set goal to be upright, and for intake to not spin
    elevator_height_ =
        0.5;  // Set the elevator to be halfway so ready for calib
  } else {
    score_goal_ = goal->score_goal();
    switch (score_goal_) {
      case HEIGHT_0:
        elevator_height_ = kElevatorFirstCube;
        wrist_angle_ = 0;
        intake_mode_ = INTAKE;
        break;
      case HEIGHT_1:
        elevator_height_ = kElevatorSecondCube;
        wrist_angle_ = 0;
        intake_mode_ = INTAKE;
        break;
      case HEIGHT_2:
        elevator_height_ = kElevatorThirdCube;
        wrist_angle_ = 0;
        intake_mode_ = INTAKE;
        break;
      case PREP_SCORE_LOW:
        elevator_height_ = kElevatorScoreLow;
        wrist_angle_ = 0;
        intake_mode_ = IDLE;
        break;
      case PREP_SCORE_MID:
        elevator_height_ = kElevatorScoreMid;
        wrist_angle_ = 0;
        intake_mode_ = IDLE;
        break;
      case PREP_SCORE_MID_BACK:
        elevator_height_ = kElevatorScoreMid;
        wrist_angle_ = M_PI * 0.9;
        intake_mode_ = IDLE;
        break;
      case PREP_SCORE_HIGH:
        elevator_height_ = kElevatorScoreHigh;
        wrist_angle_ = 0;
        intake_mode_ = IDLE;
        break;
      case PREP_SCORE_HIGH_BACK:
        elevator_height_ = kElevatorScoreHigh;
        wrist_angle_ = M_PI * 0.9;
        intake_mode_ = IDLE;
        break;
      case SCORE_LOW:
        elevator_height_ = kElevatorScoreLow;
        wrist_angle_ = 0;
        intake_mode_ = OUTTAKE;
        break;
      case SCORE_MID:
        elevator_height_ = kElevatorScoreMid;
        wrist_angle_ = 0;
        intake_mode_ = OUTTAKE;
        break;
      case SCORE_HIGH:
        elevator_height_ = kElevatorScoreHigh;
        wrist_angle_ = 0;
        intake_mode_ = OUTTAKE;
        break;
      case SCORE_MID_BACK:
        elevator_height_ = kElevatorScoreMid;
        wrist_angle_ = M_PI * 0.9;
        intake_mode_ = OUTTAKE;
        break;
      case SCORE_HIGH_BACK:
        elevator_height_ = kElevatorScoreHigh;
        wrist_angle_ = M_PI * 0.9;
        intake_mode_ = OUTTAKE;
        break;
      case IDLE_BOTTOM:
        elevator_height_ = 0;
        wrist_angle_ = 0;
        intake_mode_ = IDLE;
        break;
      case IDLE_STOW:
        elevator_height_ = 0;
        wrist_angle_ = M_PI / 2;
        intake_mode_ = IDLE;
        break;
      case INTAKE_LOW:
        has_cube_ = true;
        intake_mode_ = INTAKE;
        break;
      case OUTTAKE_LOW:
        has_cube_ = false;
        intake_mode_ = OUTTAKE;
        break;
      case IDLE_LOW:
        intake_mode_ = IDLE;
        break;
    }
  }

  if (status_->has_cube()) {
    intake_mode_ = IDLE;
  }

  // If wrist is past 80 degrees: keep elevator height above 0.9
  if (status_->wrist_angle() > (M_PI / 180.0) * 80.0) {
    elevator_height_ = muan::utils::Cap(elevator_height_, 1.9, 2);
  }
  elevator_.SetGoal(elevator_height_);

  elevator_.Update(input, &output, &status_, driver_station->is_sys_active());

  // If elevator height is below 0.9, keep wrist goal on same side of 90
  if (status_->elevator_actual_height() < 0.89 || elevator_height_ < 0.89) {
    wrist_angle_ = muan::utils::Cap(wrist_angle_, 0, (M_PI / 180.0) * 80.0);
  }

  wrist_.SetGoal(wrist_angle_, intake_mode_);
  wrist_.Update(input, &output, &status_, driver_station->is_sys_active());

  output_queue_->WriteMessage(output);
  status_queue_->WriteMessage(status_);
}

}  // namespace score_subsystem
}  // namespace c2018
