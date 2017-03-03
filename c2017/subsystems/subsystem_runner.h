#ifndef C2017_SUBSYSTEMS_SUBSYSTEM_RUNNER_H_
#define C2017_SUBSYSTEMS_SUBSYSTEM_RUNNER_H_

#include <atomic>

#include "c2017/queue_manager/queue_manager.h"
#include "c2017/wpilib/wpilib_interface.h"
#include "c2017/subsystems/drivetrain/drivetrain_base.h"
#include "c2017/vision/robot/vision.h"
#include "third_party/aos/common/time.h"
#include "third_party/aos/common/util/phased_loop.h"
#include "third_party/aos/linux_code/init.h"
#include "third_party/frc971/control_loops/drivetrain/drivetrain.h"
#include "c2017/subsystems/superstructure/superstructure.h"
#include "c2017/subsystems/lights/lights.h"

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
  c2017::lights::Lights lights_;
  c2017::vision::VisionAlignment vision_;
};

}  // namespace c2017

#endif  // C2017_SUBSYSTEMS_SUBSYSTEM_RUNNER_H_
