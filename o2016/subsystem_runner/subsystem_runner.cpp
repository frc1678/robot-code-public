#include "subsystem_runner.h"

namespace o2016 {

void SubsystemRunner::operator()() {
  aos::time::PhasedLoop phased_loop(aos::time::Time::InMS(20));

  // TODO(Kyle or Wesley) Come up with some actual value for this...
  aos::SetCurrentThreadRealtimePriority(10);
  aos::SetCurrentThreadName("SubsystemRunner");

  running_ = true;

  while (running_) {
    phased_loop.SleepUntilNext();

    //wpilib_.ReadSensors();
    // Update subsystems here
    //wpilib_.WriteActuators();
  }
}

void SubsystemRunner::Stop() {
  running_ = false;
}

}
