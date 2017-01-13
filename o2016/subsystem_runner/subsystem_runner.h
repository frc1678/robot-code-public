#ifndef O2016_SUBSYSTEMRUNNER_SUBSYSTEMRUNNER_H_
#define O2016_SUBSYSTEMRUNNER_SUBSYSTEMRUNNER_H_

#include <atomic>

#include "o2016/queue_manager/queue_manager.h"
#include "o2016/subsystems/superstructure/secondaries/queue_types.h"
#include "o2016/subsystems/superstructure/secondaries/secondaries.h"
#include "o2016/wpilib/wpilib_interface.h"
#include "third_party/aos/common/time.h"
#include "third_party/aos/common/util/phased_loop.h"
#include "third_party/aos/linux_code/init.h"

namespace o2016 {

class SubsystemRunner {
 public:
  SubsystemRunner() = default;
  ~SubsystemRunner() = default;
  void operator()();
  void Stop();

 private:
  std::atomic<bool> running_;
  secondaries::Secondaries secondaries_;
  wpilib::WpilibInterface wpilib_;
};
}
#endif
