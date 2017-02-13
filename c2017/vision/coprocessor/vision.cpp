#include "c2017/vision/coprocessor/vision.h"
#include <opencv2/opencv.hpp>
#include <memory>
#include <thread>

namespace c2017 {
namespace vision {

Vision2017::Vision2017(int camera_index)
    : range_{cv::Scalar(0, 100, 0), cv::Scalar(120, 255, 120), CV_BGR2RGB},
      constants_{1.28, 1, -.1, -.2, 1},
      vision_{range_, shared_from_this(), constants_} {
  cap_.open(camera_index);
}

double Vision2017::GetScore(double, double /* unused */, double skew, double width, double height,
                            double fullness) {
  double base_score = std::log(width * height) / (1 + std::pow(fullness - 1, 2));
  double target_score = (base_score / (1 + skew));
  return target_score;
}

void Vision2017::Morph(cv::Mat /* img */) {
  // cv::erode(img, img, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 5), cv::Point(1, 2)));
  // cv::dilate(img, img, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 31), cv::Point(0, 15)));
}

void Vision2017::operator()() {
  cv::Mat raw;
  while (true) {
    cap_ >> raw;
    muan::Vision::VisionStatus status = vision_.Update(raw);
    cv::imshow("vision", status.image_canvas);
    c2017::vision::VisionInputProto position;
    position->set_target_found(status.target_exists);
    if (status.target_exists) {
      position->set_distance_to_target(status.distance_to_target);
      position->set_angle_to_target(status.angle_to_target);
    }
    vision_queue.WriteMessage(position);
    cv::waitKey(1);
  }
}

}  // namespace vision
}  // namespace c2017
