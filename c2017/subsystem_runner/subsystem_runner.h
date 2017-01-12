#ifndef C2017_SUBSYSTEMRUNNER_SUBSYSTEMRUNNER_H_
#define C2017_SUBSYSTEMRUNNER_SUBSYSTEMRUNNER_H_

#include <atomic>

#include "c2017/queue_manager/queue_manager.h"
#include "c2017/wpilib/wpilib_interface.h"
#include "third_party/aos/common/time.h"
#include "third_party/aos/common/util/phased_loop.h"
#include "third_party/aos/linux_code/init.h"

namespace c2017 {

class SubsystemRunner {
 public:
  SubsystemRunner() = default;
  ~SubsystemRunner() = default;
  void operator()();
  void Stop();

 private:
  std::atomic<bool> running_;
  wpilib::WpilibInterface wpilib_;
};
}
#endif
