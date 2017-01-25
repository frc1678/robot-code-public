#include <opencv2/opencv.hpp>
#include <memory>
#include <alogrithm>
#include <cmath>
#include "muan/vision/vision.h"

#define VIDEO_OUTPUT 0

class ExampleVisionScorer : public muan::VisionScorer {
 public:
  double GetScore(double distance_to_target, double distance_from_previous,
                  double skew, double width, double height, double fullness) {
    // Don't limit it too much if it's really far away
    double distance_penalty = std::min(distance_from_previous, 30.0) * .08;
    // No particular reason for this, it just works most of the time
    double base_score = std::log(width * height) / (.1 + std::pow(fullness - .2, 2));
    double target_score = (base_score / (1 + skew) - distance_penalty) / (1 + distance_to_target);
    return target_score;
  }
};

int main() {
  cv::VideoCapture cap;
  cap.open("muan/vision/example/captured.avi");

#if VIDEO_OUTPUT
  cv::VideoWriter output{"output.avi", CV_FOURCC('M', 'J', 'P', 'G'), 30,
                         cv::Size(640 * 2, 480)};
#else
  cv::namedWindow("vision", cv::WINDOW_AUTOSIZE);
#endif

  muan::Vision::VisionConstants constants { 1,      // kFovX
                                            1,      // kFovY
                                            0.3,    // kCameraAngle
                                            1,      // kHeightDifference
                                            0.2 };  // kFullness

  muan::Vision::ColorRange range { cv::Scalar(50, 0, 60),
                                   cv::Scalar(100, 255, 255),
                                   CV_BGR2HSV };

  muan::Vision vision(range, std::make_shared<ExampleVisionScorer>(), constants);

  while (cap.isOpened()) {
    cv::Mat raw;
    cap >> raw;
    if (raw.empty()) {  // End of data
      break;
    }
    muan::Vision::VisionStatus status = vision.Update(raw);

#if VIDEO_OUTPUT
    cv::Mat splitscreen(image_canvas.rows, image_canvas.cols * 2, CV_8UC3);
    image_canvas.copyTo(
        splitscreen(cv::Rect(0, 0, image_canvas.cols, image_canvas.rows)));
    raw.copyTo(splitscreen(
        cv::Rect(image_canvas.cols, 0, image_canvas.cols, image_canvas.rows)));
    output.write(splitscreen);
#else
    cv::imshow("vision", status.image_canvas);
    cv::imshow("raw", raw);
    cv::waitKey(1);
#endif
  }
}
