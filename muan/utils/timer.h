#ifndef MUAN_UTILS_TIMER_H_
#define MUAN_UTILS_TIMER_H_
#include "third_party/unitscpp/unitscpp.h"

namespace muan {

class Timer {
 public:
  Timer();
  void Start();
  Time Reset();
  Time Get();

 private:
  Time start_;
};

}  // namespace muan

#endif
