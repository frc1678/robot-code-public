#ifndef C2017_VISION_COPROCESSOR_VISION_H_
#define C2017_VISION_COPROCESSOR_VISION_H_

#include <thread>
#include <chrono>
#include "c2017/vision/queue_types.h"
#include "c2017/vision/coprocessor/sender.h"
#include "muan/vision/vision.h"

namespace c2017 {
namespace vision {

class Vision2017 : public muan::VisionScorer, public std::enable_shared_from_this<Vision2017> {
 public:
  explicit Vision2017(int camera_index);
  double GetScore(double, double /* unused */, double skew, double width, double height, double fullness);
  void Morph(cv::Mat img);
  void operator()();

 protected:
  muan::Vision::ColorRange range_;
  muan::Vision::VisionConstants constants_;
  cv::VideoCapture cap_;
  muan::Vision vision_;
};

}  // namespace vision
}  // namespace c2017

#endif  // C2017_VISION_COPROCESSOR_VISION_H_
