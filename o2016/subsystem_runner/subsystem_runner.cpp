#include "subsystem_runner.h"
#include <iostream>

namespace o2016 {

SubsystemRunner::SubsystemRunner() :
    catapult_input_reader_(QueueManager::GetInstance().catapult_input_queue().MakeReader()) {}
void SubsystemRunner::operator()() {
  aos::time::PhasedLoop phased_loop(aos::time::Time::InMS(5));

  // TODO(Kyle or Wesley) Come up with some actual value for this...
  aos::SetCurrentThreadRealtimePriority(10);
  aos::SetCurrentThreadName("SubsystemRunner");

  running_ = true;

  while (running_) {
    wpilib_.ReadSensors();
    // Update subsystems here

    //o2016::secondaries::SecondariesGoalProto secondaries_goal; // Temporary
    o2016::catapult::CatapultGoalProto catapult_goal;
    catapult_goal->set_goal(o2016::catapult::CatapultGoal::SHOOT);

    catapult_.Update(catapult_input_reader_.ReadLastMessage().value(), catapult_goal, 3 * muan::units::m);
    QueueManager::GetInstance().catapult_output_queue().WriteMessage(catapult_.output());
    QueueManager::GetInstance().catapult_status_queue().WriteMessage(catapult_.status());
    //QueueManager::GetInstance().secondaries_output_queue().WriteMessage(
    //    secondaries_.Update(secondaries_goal));

    wpilib_.WriteActuators();

    phased_loop.SleepUntilNext();
  }
}

void SubsystemRunner::Stop() {
  running_ = false;
}

}
