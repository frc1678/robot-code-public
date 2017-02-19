#include "c2017/vision/coprocessor/vision.h"
#include <opencv2/opencv.hpp>
#include <memory>
#include <thread>
#include <iostream>
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
  //cv::VideoWriter output{"output.avi", CV_FOURCC('M', 'J', 'P', 'G'), 30, cv::Size(640 * 2, 480)};
  cv::VideoCapture cap;
  cap.open(camera_index);
  muan::Vision::ColorRange range{cv::Scalar(0, 30, 0), cv::Scalar(30, 160, 30), CV_BGR2RGB};
  muan::Vision::VisionConstants constants{1.14, 1, -.1, -.2, 1};
  muan::Vision vision{range, std::make_shared<VisionScorer2017>(), constants};
  cv::Mat raw;
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  while (true) {
    cap >> raw;
    muan::Vision::VisionStatus status = vision.Update(raw);
    c2017::vision::VisionInputProto position;
    position->set_target_found(status.target_exists);
    if (status.target_exists) {
      position->set_distance_to_target(status.distance_to_target);
      position->set_angle_to_target(status.angle_to_target + 0.134);
    }
    vision_queue.WriteMessage(position);
    //cv::Mat splitscreen(status.image_canvas.rows, status.image_canvas.cols * 2, CV_8UC3);
    //status.image_canvas.copyTo(splitscreen(cv::Rect(0, 0, status.image_canvas.cols, status.image_canvas.rows)));
    //raw.copyTo(splitscreen(cv::Rect(status.image_canvas.cols, 0, status.image_canvas.cols, status.image_canvas.rows)));
    //cv::imshow("vision", splitscreen);
    //cv::waitKey(1);
    //output.write(splitscreen);
    status.image_canvas.release();
    raw.release();
  }
}

}  // namespace vision
}  // namespace c2017
