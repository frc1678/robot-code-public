#include "c2019/subsystems/subsystem_runner.h"
#include "muan/utils/threading_utils.h"

namespace c2019 {
namespace subsystems {

SubsystemRunner::SubsystemRunner() {}

void SubsystemRunner::operator()() {
  aos::time::PhasedLoop phased_loop(std::chrono::milliseconds(20));
  /* aos::SetCurrentThreadRealtimePriority(10); */
  muan::utils::SetCurrentThreadName("SubsystemRunner");

  running_ = true;

  std::thread lime_thread(std::ref(limelight_));
  lime_thread.detach();

  while (running_) {
    // Subsystems go here
    interface_runner_.ReadSensors();
    drivetrain_.Update();
    superstructure_.Update();
    interface_runner_.WriteActuators();
    phased_loop.SleepUntilNext();
  }
}

}  // namespace subsystems
}  // namespace c2019
