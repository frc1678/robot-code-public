#include "vision.h"
#include <opencv2/opencv.hpp>
#include <thread>
#include "muan/vision/vision.h"

namespace c2017 {
namespace vision {

class VisionScorer2017 : public muan::VisionScorer {
 public:
  double GetScore(double , double /* unused */, double skew, double width, double height, double fullness) {
    double base_score = std::log(width * height) / (1 + std::pow(fullness - 1, 2));
    double target_score = (base_score / (1 + skew));
    return target_score;
  }

  void Morph(cv::Mat img) {
    //cv::erode(img, img, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 5), cv::Point(1, 2)));
    //cv::dilate(img, img, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 31), cv::Point(0, 15)));
  }
};

void RunVision() {
  cv::VideoCapture cap;
  cap.open(2);
  muan::Vision::ColorRange range {
    cv::Scalar(0, 100, 0), cv::Scalar(130, 255, 130), CV_BGR2RGB
  };
  muan::Vision::VisionConstants constants {1.28, 1, -.1, -.2, 1};
  muan::Vision vision{ range, std::make_shared<VisionScorer2017>(), constants};
  cv::Mat raw;
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  while(true) {
    cap >> raw;
    muan::Vision::VisionStatus status = vision.Update(raw);
    cv::imshow("vision", status.image_canvas);
    muan::vision::VisionPositionProto position;
    position->set_target_found(status.target_exists);
    if (status.target_exists) {
      position->set_target_id(0);
      position->set_distance_to_target(status.distance_to_target);
      position->set_angle_to_target(status.angle_to_target);
    }
    vision_queue.WriteMessage(position);
    cv::waitKey(1);
    status.image_canvas.release();
    raw.release();
  }
}

}
}
