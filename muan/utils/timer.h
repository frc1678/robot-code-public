#ifndef MUAN_UTILS_TIMER_H_
#define MUAN_UTILS_TIMER_H_
#include "muan/units/units.h"

namespace muan {

class Timer {
 public:
  Timer();
  void Start();
  muan::units::Time Reset();
  muan::units::Time Get();

 private:
  muan::units::Time start_;
};
}

#endif
