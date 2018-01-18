#include "c2018/subsystems/score_subsystem/score_subsystem.h"

namespace c2018 {
namespace score_subsystem {

using muan::queues::QueueManager;
using muan::wpilib::DriverStationProto;

ScoreSubsystem::ScoreSubsystem() :
                goal_reader_{ QueueManager<ScoreSubsystemGoalProto>::Fetch()->MakeReader() },
                status_queue_{ QueueManager<ScoreSubsystemStatusProto>::Fetch() },
                input_reader_{ QueueManager<ScoreSubsystemInputProto>::Fetch()->MakeReader() },
                output_queue_{ QueueManager<ScoreSubsystemOutputProto>::Fetch() },
                ds_status_reader_{ QueueManager<DriverStationProto>::Fetch()->MakeReader() }
                {}


void ScoreSubsystem::Update() {
  ScoreSubsystemGoalProto goal;
  ScoreSubsystemStatusProto status;
  ScoreSubsystemInputProto input;
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

  if (!goal->god_mode()) {
    if (!status->elevator_at_top()) {
      claw_mode_ = SCORE_F;
    } else {
      claw_mode_ = goal->claw_mode();
      if (claw_mode_ == SCORE_F) {
        claw_angle = M_PI/2;
      } else {
        claw_angle = -M_PI/2;
      }
    }
    intake_mode_ = goal->intake_mode();
    elevator_height = goal->elevator_height();

    claw_.SetGoal(claw_angle, intake_mode_);
    elevator_.SetGoal(elevator_height);
  } else {
    claw_mode_ = SCORE_F;
    
    // claw_.SetGodModeGoal(...);
    // elevator_.SetGodModeGoal(...);
  }


  claw_.Update(input, &output, &status, driver_station->is_sys_active());
  elevator_.Update(input, &output, &status, driver_station->is_sys_active());

  output_queue_->WriteMessage(output);
  status_queue_->WriteMessage(status);
}

}  // namespace score_subsystem
}  // namespace c2018
