#ifndef MUAN_UTILS_TIMING_UTILS_H_
#define MUAN_UTILS_TIMING_UTILS_H_

#include "muan/units/units.h"
#include <chrono>
#include <iostream>
#include <thread>

namespace muan {

void sleep_for(Seconds t);
Seconds now();
void sleep_until(Seconds t);
}

#endif  // MUAN_UTILS_TIMING_UTILS_H_
