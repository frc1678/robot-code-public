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
    // Update subsystems here

    // Commented because WHAT THE FUCK IS THIS
    //auto status = QueueManager::GetInstance()
                      //.driver_station_queue()
                      //.MakeReader()
                      //.ReadLastMessage();

    superstructure_.Update();
    drivetrain_.Update();

    //auto enabled = false;
    //if (status) {
      //enabled = status.value()->mode() == RobotMode::AUTONOMOUS ||
                //status.value()->mode() == RobotMode::TELEOP;
    //}

    wpilib_.WriteActuators();

    phased_loop.SleepUntilNext();
  }
}

void SubsystemRunner::Stop() { running_ = false; }
}
