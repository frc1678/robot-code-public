#ifndef MUAN_UTILS_TIMER_H_
#define MUAN_UTILS_TIMER_H_
#include "muan/units/units.h"

namespace muan {

class Timer {
 public:
  Timer();
  void Start();
  muan::units::Seconds Reset();
  muan::units::Seconds Get();

 private:
  muan::units::Seconds start_;
};
}

#endif
