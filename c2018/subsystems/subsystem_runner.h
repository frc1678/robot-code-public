#ifndef C2018_SUBSYSTEMS_SUBSYSTEM_RUNNER_H_
#define C2018_SUBSYSTEMS_SUBSYSTEM_RUNNER_H_

#include <atomic>

#include "c2018/subsystems/climber/climber.h"
#include "c2018/subsystems/drivetrain/drivetrain_base.h"
#include "c2018/wpilib/wpilib_interface.h"
#include "muan/queues/queue_manager.h"
#include "third_party/aos/common/time.h"
#include "third_party/aos/common/util/phased_loop.h"
#include "third_party/aos/linux_code/init.h"
#include "third_party/frc971/control_loops/drivetrain/drivetrain.h"
#include "c2018/subsystems/score_subsystem/score_subsystem.h"

namespace c2018 {

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
  c2018::climber::Climber climber_;
  c2018::score_subsystem::ScoreSubsystem score_subsystem_;
};

}  // namespace c2018

#endif  // C2018_SUBSYSTEMS_SUBSYSTEM_RUNNER_H_
