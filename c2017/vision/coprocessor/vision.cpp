#include "c2017/vision/coprocessor/vision.h"
#include <fcntl.h>
#include <google/protobuf/text_format.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <opencv2/opencv.hpp>
#include <memory>
#include <thread>
#include "muan/vision/vision.h"

#define VIDEO_OUTPUT_SCREEN 1
#define VIDEO_OUTPUT_FILE 0

namespace c2017 {
namespace vision {

double VisionScorer2017::GetScore(double /* distance_to_target */, double /* distance_from_previous */,
                                  double skew, double width, double height, double fullness) {
  double base_score = std::log(width * height) / (1 + std::pow(fullness - 1, 2));
  double target_score = (base_score / (1 + skew));
  return target_score;
}

void RunVision(int camera_index) {
  // Read from file
  VisionConstants robot_constants;
  VisionThresholds thresholds;

  {
    int file = open("c2017/vision/coprocessor/robot_constants.pb.text", O_RDONLY);
    google::protobuf::io::FileInputStream fstream(file);
    google::protobuf::TextFormat::Parse(&fstream, &robot_constants);
  }

  {
    int file = open("c2017/vision/coprocessor/thresholds.pb.text", O_RDONLY);
    google::protobuf::io::FileInputStream fstream(file);
    google::protobuf::TextFormat::Parse(&fstream, &thresholds);
  }

#if VIDEO_OUTPUT_FILE
  cv::VideoWriter output{"output.avi", CV_FOURCC('M', 'J', 'P', 'G'), 30, cv::Size(640 * 2, 480)};
#endif
  cv::VideoCapture cap;
  cap.open(camera_index);

  int colorspace_code = 0;
  switch (thresholds.space()) {
    case VisionThresholds::Rgb:
      colorspace_code = CV_BGR2RGB;
      break;
    case VisionThresholds::Hsv:
      colorspace_code = CV_BGR2HSV;
      break;
    default:
      break;
  }

  muan::Vision::ColorRange range{cv::Scalar(thresholds.a_low(),
                                            thresholds.b_low(),
                                            thresholds.c_low()),
                                 cv::Scalar(thresholds.a_high(),
                                            thresholds.b_high(),
                                            thresholds.c_high()),
                                 colorspace_code};

  muan::Vision::VisionConstants constants{1.14,  // FOV is not different per robot
                                          0.659,
                                          robot_constants.x_camera_angle(),
                                          robot_constants.y_camera_angle(),
                                          1.66,  // Field properties are
                                          1.,    // not different per robot
                                          0.0005};

  muan::Vision vision{range, std::make_shared<VisionScorer2017>(), constants};
  cv::Mat raw;
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  while (true) {
    cap >> raw;
    // Don't detect the bottom 1/3rd of the image. The target isn't there,
    // and there is often more noise and reflective robot parts.
    cv::rectangle(raw, cv::Point(0, raw.rows), cv::Point(raw.cols, raw.rows*2/3),
                  cv::Scalar(0, 0, 0), -1, 8, 0);
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
  }
}

}  // namespace vision
}  // namespace c2017
