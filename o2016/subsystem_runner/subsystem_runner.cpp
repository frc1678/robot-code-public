#include "subsystem_runner.h"

namespace o2016 {

void SubsystemRunner::operator()() {
  aos::time::PhasedLoop phased_loop(aos::time::Time::InMS(5));

  // TODO(Kyle or Wesley) Come up with some actual value for this...
  aos::SetCurrentThreadRealtimePriority(10);
  aos::SetCurrentThreadName("SubsystemRunner");

  running_ = true;

  while (running_) {
    wpilib_.ReadSensors();
    // Update subsystems here

    o2016::secondaries::SecondariesGoalProto secondaries_goal;  // Temporary
    o2016::turret::TurretGoalProto turret_goal;                 // Temporary

    auto status = QueueManager::GetInstance()
                      .driver_station_queue()
                      .MakeReader()
                      .ReadLastMessage();

    auto intake_inputs = QueueManager::GetInstance()
                             .intake_input_queue()
                             .MakeReader()
                             .ReadLastMessage()
                             .value();

    turret_goal->set_goal_angle(0);
    turret_.SetGoal(turret_goal);

    intake::IntakeGoalProto intake_goal;
    intake_goal->set_goal_angle(0);

    o2016::secondaries::SecondariesGoalProto secondaries_goal;  // Temporary

    QueueManager::GetInstance().secondaries_output_queue().WriteMessage(
        secondaries_.Update(secondaries_goal));
    QueueManager::GetInstance().turret_output_queue().WriteMessage(
        turret_.Update(turret_input_.ReadLastMessage().value()));

    drivetrain_.Update();

    auto enabled = false;
    if (status) {
      enabled = status.value()->mode() == RobotMode::AUTONOMOUS ||
                status.value()->mode() == RobotMode::TELEOP;
    }

    QueueManager::GetInstance().intake_output_queue().WriteMessage(
        intake_.Update(intake_inputs, intake_goal, enabled));

    wpilib_.WriteActuators();

    phased_loop.SleepUntilNext();
  }
}

void SubsystemRunner::Stop() { running_ = false; }
}
