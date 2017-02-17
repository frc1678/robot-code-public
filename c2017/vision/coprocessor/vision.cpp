#include "c2017/vision/coprocessor/vision.h"
#include <opencv2/opencv.hpp>
#include <memory>
#include <thread>
#include "muan/vision/vision.h"

namespace c2017 {
namespace vision {

double VisionScorer2017::GetScore(double, double /* unused */, double skew,
                                  double width, double height, double fullness) {
  double base_score = std::log(width * height) / (1 + std::pow(fullness - 1, 2));
  double target_score = (base_score / (1 + skew));
  return target_score;
}

void RunVision(int camera_index) {
  cv::VideoCapture cap;
  cap.open(camera_index);
  muan::Vision::ColorRange range{cv::Scalar(0, 100, 0), cv::Scalar(120, 255, 120), CV_BGR2RGB};
  muan::Vision::VisionConstants constants{1.28, 1, -.1, -.2, 1};
  muan::Vision vision{range, std::make_shared<VisionScorer2017>(), constants};
  cv::Mat raw;
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  while (true) {
    cap >> raw;
    muan::Vision::VisionStatus status = vision.Update(raw);
    cv::imshow("vision", status.image_canvas);
    c2017::vision::VisionInputProto position;
    position->set_target_found(status.target_exists);
    if (status.target_exists) {
      position->set_distance_to_target(status.distance_to_target);
      position->set_angle_to_target(status.angle_to_target);
    }
    vision_queue.WriteMessage(position);
    cv::waitKey(1);
    status.image_canvas.release();
    raw.release();
  }
}

}  // namespace vision
}  // namespace c2017
