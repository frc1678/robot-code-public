#include "muan/wpilib/can_wrapper.h"
#include "muan/utils/threading_utils.h"
#include "third_party/aos/common/time.h"
#include "third_party/aos/common/util/phased_loop.h"
#include "third_party/aos/linux_code/init.h"

namespace muan {
namespace wpilib {

CanWrapper::CanWrapper(PdpWrapper::Queue* pdp_queue) {
  /* pdp_.SetQueue(pdp_queue); */
}

void CanWrapper::operator()() {
  aos::time::PhasedLoop phased_loop(std::chrono::milliseconds(20));

  // TODO(Kyle) Come up with some actual value for this...
  aos::SetCurrentThreadRealtimePriority(10);
  muan::utils::SetCurrentThreadName("CanWrapper");

  running_ = true;

  while (running_) {
    phased_loop.SleepUntilNext();
    /* pdp_.SendValues(); */
    pcm_.Flush();
  }
}

void CanWrapper::Stop() { running_ = false; }

/* PdpWrapper* CanWrapper::pdp() { return &pdp_; } */
PcmWrapper* CanWrapper::pcm() { return &pcm_; }

}  // namespace wpilib
}  // namespace muan
