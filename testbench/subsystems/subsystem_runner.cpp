#include "testbench/subsystems/subsystem_runner.h"
#include "testbench/queue_manager/queue_manager.h"
#include "testbench/subsystems/drivetrain/drivetrain_base.h"

namespace testbench {

SubsystemRunner::SubsystemRunner()
    : drivetrain_{::testbench::drivetrain::GetDrivetrainConfig(),
                  QueueManager::GetInstance()->drivetrain_goal_queue(),
                  QueueManager::GetInstance()->drivetrain_input_queue(),
                  QueueManager::GetInstance()->drivetrain_output_queue(),
                  QueueManager::GetInstance()->drivetrain_status_queue(),
                  QueueManager::GetInstance()->driver_station_queue(),
                  QueueManager::GetInstance()->gyro_queue()} {}

void SubsystemRunner::operator()() {
  aos::time::PhasedLoop phased_loop(aos::time::Time::InMS(5));

  // TODO(Kyle or Wesley) Come up with some actual value for this...
  aos::SetCurrentThreadRealtimePriority(10);
  aos::SetCurrentThreadName("SubsystemRunner");

  running_ = true;

  while (running_) {
    wpilib_.ReadSensors();

    // Update subsystems here
    drivetrain_.Update();

    wpilib_.WriteActuators();

    phased_loop.SleepUntilNext();
  }
}

void SubsystemRunner::Stop() { running_ = false; }

}  // testbench
