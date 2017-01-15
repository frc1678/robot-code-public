#ifndef C2017_VISION_COPROCESSOR_SENDER_H_
#define C2017_VISION_COPROCESSOR_SENDER_H_

#include <thread>
#include <chrono>
#include <iostream>
#include "third_party/aos/vision/events/udp.h"
#include "muan/vision/queue_types.h"

namespace c2017 {
namespace vision {

void RunReader();

} // namespace vision
} // namespace c2017

#endif // C2017_VISION_COPROCESSOR_SENDER_H_
