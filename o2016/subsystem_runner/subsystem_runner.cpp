#include "subsystem_runner.h"

namespace o2016 {

void SubsystemRunner::operator()() {
  aos::time::PhasedLoop phased_loop(aos::time::Time::InMS(5));

  // TODO(Kyle or Wesley) Come up with some actual value for this...
  aos::SetCurrentThreadRealtimePriority(10);
  aos::SetCurrentThreadName("SubsystemRunner");

  running_ = true;

  QueueManager::GetInstance().intake_goal_queue().WriteMessage(
      o2016::intake::IntakeGoalProto{});

  while (running_) {
    wpilib_.ReadSensors();
    // Update subsystems here

    o2016::turret::TurretGoalProto turret_goal;  // Temporary

    auto status = QueueManager::GetInstance()
                      .driver_station_queue()
                      .MakeReader()
                      .ReadLastMessage();

    auto intake_inputs = QueueManager::GetInstance()
                             .intake_input_queue()
                             .MakeReader()
                             .ReadLastMessage()
                             .value();

    auto secondaries_goal = QueueManager::GetInstance()
                                .secondaries_goal_queue()
                                .MakeReader()
                                .ReadLastMessage();

    /* turret_goal->set_goal_angle(0); */
    /* turret_.SetGoal(turret_goal); */

    auto intake_goal = QueueManager::GetInstance()
                           .intake_goal_queue()
                           .MakeReader()
                           .ReadLastMessage()
                           .value();

    if (secondaries_goal) {
      QueueManager::GetInstance().secondaries_output_queue().WriteMessage(
          secondaries_.Update(*secondaries_goal));
    }
    /* QueueManager::GetInstance().turret_output_queue().WriteMessage( */
    /*     turret_.Update(turret_input_.ReadLastMessage().value())); */

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
