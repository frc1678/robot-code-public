#ifndef TESTBENCH_SUBSYSTEMS_SUBSYSTEMRUNNER_H_
#define TESTBENCH_SUBSYSTEMS_SUBSYSTEMRUNNER_H_

#include <atomic>

#include "testbench/queue_manager/queue_manager.h"
#include "testbench/wpilib/wpilib_interface.h"
#include "third_party/aos/common/time.h"
#include "third_party/aos/common/util/phased_loop.h"
#include "third_party/aos/linux_code/init.h"
#include "third_party/frc971/control_loops/drivetrain/drivetrain.h"

namespace testbench {

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
};

}  // testbench

#endif  // TESTBENCH_SUBSYSTEMS_SUBSYSTEMRUNNER_H_
