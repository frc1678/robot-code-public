#ifndef C2017_SUBSYSTEMS_SUBSYSTEMRUNNER_H_
#define C2017_SUBSYSTEMS_SUBSYSTEMRUNNER_H_

#include <atomic>

#include "c2017/queue_manager/queue_manager.h"
#include "c2017/wpilib/wpilib_interface.h"
#include "c2017/subsystems/drivetrain/drivetrain_base.h"
#include "third_party/aos/common/time.h"
#include "third_party/aos/common/util/phased_loop.h"
#include "third_party/aos/linux_code/init.h"
#include "third_party/frc971/control_loops/drivetrain/drivetrain.h"
#include "c2017/subsystems/superstructure/superstructure.h"

namespace c2017 {

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
  c2017::superstructure::SuperStructure superstructure_;
};
}
#endif
