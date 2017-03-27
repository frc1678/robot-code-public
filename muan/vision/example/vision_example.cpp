#include <opencv2/opencv.hpp>
#include <algorithm>
#include <memory>
#include <cmath>
#include "muan/vision/vision.h"

#define VIDEO_OUTPUT 0

int main() {
  cv::VideoCapture cap;
  cap.open("muan/vision/example/captured.avi");

#if VIDEO_OUTPUT
  cv::VideoWriter output{"output.avi", CV_FOURCC('M', 'J', 'P', 'G'), 30, cv::Size(640 * 2, 480)};
#else
  cv::namedWindow("vision", cv::WINDOW_AUTOSIZE);
#endif

  muan::vision::VisionConstants constants{1,       // kFovX
                                          1,       // kFovY
                                          0,       // kCameraAngleX
                                          0.3,     // kCameraAngleY
                                          0.0005,  // kMinTargetArea
                                          0.09};   // kMaxTargetArea

  muan::vision::VisionThresholds range;
  range.set_a_low(50);
  range.set_b_low(0);
  range.set_c_low(60);
  range.set_a_high(100);
  range.set_b_high(255);
  range.set_c_high(255);
  muan::vision::Vision vision(range, constants);

  while (cap.isOpened()) {
    cv::Mat raw;
    cap >> raw;
    if (raw.empty()) {  // End of data
      break;
    }
    cv::Mat image_canvas = raw.clone();
    auto targets = vision.Update(raw, image_canvas);

#if VIDEO_OUTPUT
    cv::Mat splitscreen(image_canvas.rows, image_canvas.cols * 2, CV_8UC3);
    image_canvas.copyTo(splitscreen(cv::Rect(0, 0, image_canvas.cols, image_canvas.rows)));
    raw.copyTo(splitscreen(cv::Rect(image_canvas.cols, 0, image_canvas.cols, image_canvas.rows)));
    output.write(splitscreen);
#else
    cv::imshow("vision", image_canvas);
    cv::imshow("raw", raw);
    cv::waitKey(1);
#endif
  }
}
