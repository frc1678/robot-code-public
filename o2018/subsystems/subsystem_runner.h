#ifndef O2018_SUBSYSTEMS_SUBSYSTEM_RUNNER_H_
#define O2018_SUBSYSTEMS_SUBSYSTEM_RUNNER_H_

#include <atomic>

#include "third_party/aos/common/time.h"
#include "third_party/aos/common/util/phased_loop.h"
#include "third_party/aos/linux_code/init.h"
#include "o2018/subsystems/arm/arm.h"
#include "muan/subsystems/drivetrain/drivetrain.h"
#include "o2018/interfaces/interface_runner.h"
#include "o2018/subsystems/drivetrain/drivetrain_base.h"

namespace o2018 {
namespace subsystems {

class SubsystemRunner {
 public:
  SubsystemRunner();
  ~SubsystemRunner() = default;
  void operator()();

 private:
  // Subsystems go here
  arm::Arm arm_;
  muan::subsystems::drivetrain::Drivetrain drivetrain_{o2018::subsystems::drivetrain::GetDrivetrainConfig()};
  interfaces::InterfaceRunner interface_runner_;
  std::atomic<bool> running_;
};

}  // namespace subsystems
}  // namespace o2018

#endif  // O2018_SUBSYSTEMS_SUBSYSTEM_RUNNER_H_
