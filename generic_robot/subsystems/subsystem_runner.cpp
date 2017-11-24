#include "generic_robot/subsystems/subsystem_runner.h"
#include "muan/utils/threading_utils.h"

namespace generic_robot {

SubsystemRunner::SubsystemRunner()
    : drivetrain_{::generic_robot::drivetrain::GetDrivetrainConfig(),
                  QueueManager::GetInstance()->drivetrain_goal_queue(),
                  QueueManager::GetInstance()->drivetrain_input_queue(),
                  QueueManager::GetInstance()->drivetrain_output_queue(),
                  QueueManager::GetInstance()->drivetrain_status_queue(),
                  QueueManager::GetInstance()->driver_station_queue(),
                  QueueManager::GetInstance()->gyro_queue()} {}

void SubsystemRunner::operator()() {
  aos::time::PhasedLoop phased_loop(std::chrono::milliseconds(5));

  // TODO(Kyle or Wesley) Come up with some actual value for this...
  aos::SetCurrentThreadRealtimePriority(50);
  muan::utils::SetCurrentThreadName("SubsystemRunner");

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

}  // namespace generic_robot
