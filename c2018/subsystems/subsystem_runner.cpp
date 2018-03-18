#include "c2018/subsystems/subsystem_runner.h"
#include "muan/utils/threading_utils.h"
#include "muan/wpilib/gyro/gyro_reader.h"

namespace c2018 {

using DrivetrainGoalProto = frc971::control_loops::drivetrain::GoalProto;
using DrivetrainInputProto = frc971::control_loops::drivetrain::InputProto;
using DrivetrainOutputProto = frc971::control_loops::drivetrain::OutputProto;
using DrivetrainStatusProto = frc971::control_loops::drivetrain::StatusProto;
using muan::queues::QueueManager;
using muan::wpilib::DriverStationProto;
using muan::wpilib::gyro::GyroMessageProto;

SubsystemRunner::SubsystemRunner()
    : drivetrain_{::c2018::drivetrain::GetDrivetrainConfig(),
                  QueueManager<DrivetrainGoalProto>::Fetch(),
                  QueueManager<DrivetrainInputProto>::Fetch(),
                  QueueManager<DrivetrainOutputProto>::Fetch(),
                  QueueManager<DrivetrainStatusProto>::Fetch(),
                  QueueManager<DriverStationProto>::Fetch(),
                  QueueManager<GyroMessageProto>::Fetch()} {}

void SubsystemRunner::operator()() {
  aos::time::PhasedLoop phased_loop(std::chrono::milliseconds(5));

  aos::SetCurrentThreadRealtimePriority(50);
  muan::utils::SetCurrentThreadName("SubsystemRunner");

  running_ = true;

  while (running_) {
    wpilib_.ReadSensors();
    // Update subsystems here

    drivetrain_.Update();

    climber_.Update();

    score_subsystem_.Update();

    wpilib_.WriteActuators();

    phased_loop.SleepUntilNext();
  }
}

void SubsystemRunner::Stop() { running_ = false; }

}  // namespace c2018
