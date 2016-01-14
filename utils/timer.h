#ifndef MUAN_UTILS_TIMER_H_
#define MUAN_UTILS_TIMER_H_
#include "../unitscpp/unitscpp.h"

class Timer {
 public:
  Timer();
  void Start();
  void Reset();
  Time Get();

 private:
  Time start_;
};

#endif
