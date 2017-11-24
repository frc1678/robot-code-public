#ifndef O2017_SUBSYSTEMS_SUBSYSTEM_RUNNER_H_
#define O2017_SUBSYSTEMS_SUBSYSTEM_RUNNER_H_

#include <atomic>

#include "o2017/queue_manager/queue_manager.h"
#include "o2017/subsystems/drivetrain/drivetrain_base.h"
#include "o2017/subsystems/superstructure/superstructure.h"
#include "o2017/wpilib/wpilib_interface.h"
#include "third_party/aos/common/time.h"
#include "third_party/aos/common/util/phased_loop.h"
#include "third_party/aos/linux_code/init.h"
#include "third_party/frc971/control_loops/drivetrain/drivetrain.h"

namespace o2017 {

class SubsystemRunner {
 public:
  SubsystemRunner();
  ~SubsystemRunner() = default;
  void operator()();
  void Stop();

 private:
  std::atomic<bool> running_;
  wpilib::WpilibInterface wpilib_;
  frc971::control_loops::drivetrain::DrivetrainLoop drivetrain_;
  o2017::superstructure::Superstructure superstructure_;
};

}  // namespace o2017

#endif  // O2017_SUBSYSTEMS_SUBSYSTEM_RUNNER_H_
