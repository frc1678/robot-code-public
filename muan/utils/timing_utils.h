#ifndef MUAN_UTILS_TIMING_UTILS_H_
#define MUAN_UTILS_TIMING_UTILS_H_

#include "third_party/unitscpp/unitscpp.h"
#include <chrono>
#include <iostream>
#include <thread>

namespace muan {

void sleep_for(Time t);
Time now();
void sleep_until(Time t);

}  // namespace muan

#endif /* MUAN_UTILS_TIMING_UTILS_H_ */
