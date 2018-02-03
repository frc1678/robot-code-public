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
  ScoreSubsystemStatusProto status;
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
    wrist_angle = 0;
    intake_mode = IDLE;
    // Set goal to be upright, and for intake to not spin
    elevator_height = 0.5;  // Set the elevator to be halfway so ready for calib
  } else {
    switch (goal->score_goal()) {
      case HEIGHT_0:
        elevator_height = kElevatorFirstCube;
        wrist_angle = M_PI / 2;
        intake_mode = INTAKE;
        break;
      case HEIGHT_1:
        elevator_height = kElevatorSecondCube;
        wrist_angle = M_PI / 2;
        intake_mode = INTAKE;
        break;
      case HEIGHT_2:
        elevator_height = kElevatorThirdCube;
        wrist_angle = M_PI / 2;
        intake_mode = INTAKE;
        break;
      case PREP_SCORE_LOW:
        elevator_height = kElevatorScoreLow;
        wrist_angle = 0;
        intake_mode = IDLE;
        break;
      case PREP_SCORE_HIGH:
        elevator_height = kElevatorScoreHigh;
        wrist_angle = 0;
        intake_mode = IDLE;
        break;
      case PREP_SCORE_HIGH_BACK:
        elevator_height = kElevatorScoreHigh;
        wrist_angle = M_PI;
        intake_mode = IDLE;
        break;
      case SCORE_LOW:
        elevator_height = kElevatorScoreLow;
        wrist_angle = 0;
        intake_mode = OUTTAKE;
        break;
      case SCORE_HIGH:
        elevator_height = kElevatorScoreHigh;
        wrist_angle = 0;
        intake_mode = OUTTAKE;
        break;
      case SCORE_HIGH_BACK:
        elevator_height = kElevatorScoreHigh;
        wrist_angle = M_PI;
        intake_mode = OUTTAKE;
        break;
      case IDLE_BOTTOM:
        elevator_height = 0;
        wrist_angle = 0;
        intake_mode = IDLE;
        break;
      case IDLE_STOW:
        elevator_height = 0;
        wrist_angle = M_PI / 2;
        intake_mode = IDLE;
    }
  }

  if (!status->elevator_at_top()) {
    wrist_angle = muan::utils::Cap(wrist_angle, 0, M_PI / 2);
  }

  elevator_.SetGoal(elevator_height);
  wrist_.SetGoal(wrist_angle, intake_mode);

  elevator_.Update(input, &output, &status, driver_station->is_sys_active());
  wrist_.Update(input, &output, &status, driver_station->is_sys_active());
  output_queue_->WriteMessage(output);
  status_queue_->WriteMessage(status);
}

}  // namespace score_subsystem
}  // namespace c2018
