#include "subsystem_runner.h"
#include <iostream>

namespace o2016 {


void SubsystemRunner::operator()() {
  aos::time::PhasedLoop phased_loop(aos::time::Time::InMS(5));

  // TODO(Kyle or Wesley) Come up with some actual value for this...
  aos::SetCurrentThreadRealtimePriority(10);
  aos::SetCurrentThreadName("SubsystemRunner");

  running_ = true;

  while (running_) {
    wpilib_.ReadSensors();

    auto status = ds_reader_.ReadLastMessage();
    auto enabled = false;
    if (status) {
      enabled = status.value()->mode() == RobotMode::AUTONOMOUS ||
                status.value()->mode() == RobotMode::TELEOP;
    }

    superstructure_.Update(enabled);
    drivetrain_.Update();

    wpilib_.WriteActuators();

    phased_loop.SleepUntilNext();
  }
}

void SubsystemRunner::Stop() { running_ = false; }
}
