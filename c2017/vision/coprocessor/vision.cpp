#include "c2017/vision/coprocessor/vision.h"
#include <opencv2/opencv.hpp>
#include <memory>
#include <thread>
#include <iostream>
#include "muan/vision/vision.h"

#define VIDEO_OUTPUT_SCREEN 0
#define VIDEO_OUTPUT_FILE 1

namespace c2017 {
namespace vision {

double VisionScorer2017::GetScore(double /* distance_to_target */, double /* distance_from_previous */,
                                  double skew, double width, double height, double fullness) {
  double base_score = std::log(width * height) / (1 + std::pow(fullness - 1, 2));
  double target_score = (base_score / (1 + skew));
  return target_score;
}

void RunVision(int camera_index) {
#if VIDEO_OUTPUT_FILE
  cv::VideoWriter output{"output.avi", CV_FOURCC('M', 'J', 'P', 'G'), 30, cv::Size(640 * 2, 480)};
#endif
  cv::VideoCapture cap;
  cap.open(camera_index);
  muan::Vision::ColorRange range{cv::Scalar(0, 50, 0), cv::Scalar(50, 180, 50), CV_BGR2RGB};
  muan::Vision::VisionConstants constants{1.14, 0.659, 0.134, 0.524, 1.66, 1., 0.0005};
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
      position->set_angle_to_target(status.angle_to_target);
    }
    vision_queue.WriteMessage(position);
#if VIDEO_OUTPUT_FILE || VIDEO_OUTPUT_SCREEN
    cv::Mat splitscreen(status.image_canvas.rows, status.image_canvas.cols * 2, CV_8UC3);
    status.image_canvas.copyTo(
        splitscreen(cv::Rect(0, 0, status.image_canvas.cols, status.image_canvas.rows)));
    raw.copyTo(splitscreen(
        cv::Rect(status.image_canvas.cols, 0, status.image_canvas.cols, status.image_canvas.rows)));
#endif
#if VIDEO_OUTPUT_SCREEN
    cv::imshow("vision", splitscreen);
    cv::waitKey(1);
#endif
#if VIDEO_OUTPUT
    output.write(splitscreen);
#endif
    status.image_canvas.release();
    raw.release();
  }
}

}  // namespace vision
}  // namespace c2017
