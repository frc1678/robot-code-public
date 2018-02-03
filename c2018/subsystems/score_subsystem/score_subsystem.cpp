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

  if (!goal_reader_.ReadLastMessage(&goal)) {
    // Set default goal
  }

  if (!ds_status_reader_.ReadLastMessage(&driver_station)) {
    driver_station->set_battery_voltage(12.0);
  }

  if (!status->elevator_at_top()) {
    elevator_height = goal->elevator_height();

    elevator_.SetGoal(elevator_height);
    wrist_.SetGoal(goal->wrist_angle(), goal->intake_mode());
    std::cout << "Set wrist goal" << std::endl;
  } else {
    // claw_mode_ = SCORE_F;

    // claw_.SetGodModeGoal(...);   doesn't exist yet
    wrist_.SetGoal(goal->wrist_angle(), goal->intake_mode());
    std::cout << "Set wrist goal" << std::endl;
  }

  elevator_.Update(input, &output, &status, driver_station->is_sys_active());
  wrist_.Update(input, &output, &status, driver_station->is_sys_active());
  output_queue_->WriteMessage(output);
  status_queue_->WriteMessage(status);
}

}  // namespace score_subsystem
}  // namespace c2018
