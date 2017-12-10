#include "third_party/aos/common/condition.h"

#include <inttypes.h>
#include <assert.h>

#include "third_party/aos/common/mutex.h"

namespace aos {

Condition::Condition(Mutex *m) : impl_(), m_(m) {}

bool Condition::Wait() {
  const int ret = condition_wait(&impl_, &m_->impl_);
  assert(__builtin_expect(ret == 0 || ret == 1, 1));
  return ret == 1;
}

void Condition::Signal() {
  condition_signal(&impl_, &m_->impl_);
}

void Condition::Broadcast() {
  condition_broadcast(&impl_, &m_->impl_);
}

}  // namespace aos
