#ifndef C2017_VISION_COPROCESSOR_SENDER_H_
#define C2017_VISION_COPROCESSOR_SENDER_H_

#include <thread>
#include <chrono>
#include <iostream>
#include "third_party/aos/vision/events/udp.h"
#include "c2017/vision/queue_types.h"

namespace c2017 {
namespace vision {

void RunSender(char* target_ip);
extern c2017::vision::VisionInputQueue vision_queue;
}
}

#endif  // C2017_VISION_COPROCESSOR_SENDER_H_
