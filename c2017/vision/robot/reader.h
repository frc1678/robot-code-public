#ifndef C2017_VISION_COPROCESSOR_SENDER_H_
#define C2017_VISION_COPROCESSOR_SENDER_H_

#include <thread>
#include <chrono>
#include <iostream>
#include "third_party/aos/vision/events/udp.h"
#include "muan/vision/queue_types.h"
#include "third_party/aos/common/time.h"
#include "third_party/aos/common/util/phased_loop.h"
#include "third_party/aos/linux_code/init.h"

namespace c2017 {
namespace vision {

class VisionReader {
 public:
  VisionReader();
  ~VisionReader() = default;
  void operator()();
  void Stop();
 private:
  bool running_;
};

} // namespace vision
} // namespace c2017

#endif // C2017_VISION_COPROCESSOR_SENDER_H_
