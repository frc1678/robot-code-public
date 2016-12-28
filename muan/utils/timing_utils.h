#ifndef MUAN_UTILS_TIMING_UTILS_H_
#define MUAN_UTILS_TIMING_UTILS_H_

#include <chrono>
#include <iostream>
#include <thread>
#include "muan/units/units.h"

namespace muan {
namespace utils {

void sleep_for(muan::units::Time t);
muan::units::Time now();
void sleep_until(muan::units::Time t);

}  // utils
}  // muan

#endif /* MUAN_UTILS_TIMING_UTILS_H_ */
