#ifndef MUAN_UTILS_TIMING_UTILS_H_
#define MUAN_UTILS_TIMING_UTILS_H_

#include <chrono>
#include <thread>
#include "muan/unitscpp/unitscpp.h"
#include <iostream>

namespace muan {

void sleep_for(Time t);
Time now();
void sleep_until(Time t);
}

#endif  // MUAN_UTILS_TIMING_UTILS_H_
