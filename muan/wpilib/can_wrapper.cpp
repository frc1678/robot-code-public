#include "can_wrapper.h"
#include "third_party/aos/common/time.h"
#include "third_party/aos/common/util/phased_loop.h"
#include "third_party/aos/linux_code/init.h"

namespace muan {

namespace wpilib {

CanWrapper::CanWrapper(PdpWrapper::Queue* pdp_queue) { pdp_.SetQueue(pdp_queue); }

void CanWrapper::operator()() {
  aos::time::PhasedLoop phased_loop(aos::time::Time::InMS(20));

  // TODO(Kyle) Come up with some actual value for this...
  aos::SetCurrentThreadRealtimePriority(10);
  aos::SetCurrentThreadName("CanWrapper");

  running_ = true;

  while (running_) {
    int iterations_passed = phased_loop.SleepUntilNext();
    pdp_.SendValues();
    pcm_.Flush();
  }
}

void CanWrapper::Stop() { running_ = false; }

PdpWrapper* CanWrapper::pdp() { return &pdp_; }

PcmWrapper* CanWrapper::pcm() { return &pcm_; }

}  // wpilib

}  // muan
