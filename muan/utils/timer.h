#ifndef MUAN_UTILS_TIMER_H_
#define MUAN_UTILS_TIMER_H_
#include "muan/units/units.h"

namespace muan {

class Timer {
  using namespace muan::units;

 public:
  Timer();
  void Start();
  Seconds Reset();
  Seconds Get();

 private:
  Seconds start_;
};
}

#endif
