#ifndef MUAN_UTILS_TIMING_UTILS_H_
#define MUAN_UTILS_TIMING_UTILS_H_

#include "muan/units/units.h"
#include <chrono>
#include <iostream>
#include <thread>

namespace muan {

void sleep_for(muan::units::Time t);
muan::units::Time now();
void sleep_until(muan::units::Time t);
}

#endif  // MUAN_UTILS_TIMING_UTILS_H_
