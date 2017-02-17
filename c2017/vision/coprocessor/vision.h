#ifndef C2017_VISION_COPROCESSOR_VISION_H_
#define C2017_VISION_COPROCESSOR_VISION_H_

#include <thread>
#include <chrono>
#include "c2017/vision/queue_types.h"
#include "c2017/vision/coprocessor/sender.h"
#include "muan/vision/vision.h"

namespace c2017 {
namespace vision {

class VisionScorer2017 : public muan::VisionScorer {
 public:
  double GetScore(double, double /* unused */, double skew, double width,
                  double height, double fullness);
  void Morph(cv::Mat img);
};

void RunVision(int camera_index);

}  // namespace vision
}  // namespace c2017

#endif  // C2017_VISION_COPROCESSOR_VISION_H_
