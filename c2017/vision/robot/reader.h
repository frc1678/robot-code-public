#ifndef C2017_VISION_ROBOT_READER_H_
#define C2017_VISION_ROBOT_READER_H_

#include "c2017/vision/queue_types.h"

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
  VisionInputQueue& vision_input_queue_;
};

}  // namespace vision
}  // namespace c2017

#endif  // C2017_VISION_ROBOT_READER_H_
