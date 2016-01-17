#ifndef MUAN_UTILS_TIMING_UTILS_H_
#define MUAN_UTILS_TIMING_UTILS_H_

#include <chrono>
#include <thread>
#include "../unitscpp/unitscpp.h"
#include <iostream>

namespace muan {

void sleep_for(Time t) {
  std::this_thread::sleep_for(std::chrono::milliseconds((int)t.to(ms)));
}

Time now() {
  using namespace std::chrono;
  auto since_epoch = system_clock::now().time_since_epoch();
  return duration_cast<microseconds>(since_epoch).count() * us;
}

void sleep_until(Time t) {
  using namespace std::chrono;
  auto as_time_point =
      time_point<system_clock>(microseconds(static_cast<long long>(t.to(us))));
  std::this_thread::sleep_until(as_time_point);
}

}

#endif  // MUAN_UTILS_TIMING_UTILS_H_
