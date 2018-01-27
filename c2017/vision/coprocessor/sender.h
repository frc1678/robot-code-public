#ifndef C2017_VISION_COPROCESSOR_SENDER_H_
#define C2017_VISION_COPROCESSOR_SENDER_H_

#include <chrono>
#include <iostream>
#include <thread>
#include "c2017/vision/queue_types.h"
#include "third_party/aos/vision/events/udp.h"

namespace c2017 {
namespace vision {

void RunSender(const char* target_ip);
extern c2017::vision::VisionInputQueue vision_queue;
}
}

#endif  // C2017_VISION_COPROCESSOR_SENDER_H_
