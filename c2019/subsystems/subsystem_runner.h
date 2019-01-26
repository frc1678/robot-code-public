#ifndef C2019_SUBSYSTEMS_SUBSYSTEM_RUNNER_H_
#define C2019_SUBSYSTEMS_SUBSYSTEM_RUNNER_H_

#include <atomic>

#include "muan/subsystems/drivetrain/drivetrain.h"
#include "c2019/interfaces/interface_runner.h"
#include "c2019/subsystems/drivetrain/drivetrain_base.h"
#include "c2019/subsystems/superstructure/superstructure.h"
#include "third_party/aos/common/time.h"
#include "third_party/aos/common/util/phased_loop.h"
#include "third_party/aos/linux_code/init.h"

namespace c2019 {
namespace subsystems {

class SubsystemRunner {
 public:
  SubsystemRunner();
  ~SubsystemRunner() = default;
  void operator()();

 private:
  // Subsystems go here
  muan::subsystems::drivetrain::Drivetrain drivetrain_{
      c2019::subsystems::drivetrain::GetDrivetrainConfig()};
  superstructure::Superstructure superstructure_;
  interfaces::InterfaceRunner interface_runner_;
  std::atomic<bool> running_;
};

}  // namespace subsystems
}  // namespace c2019

#endif  // C2019_SUBSYSTEMS_SUBSYSTEM_RUNNER_H_
