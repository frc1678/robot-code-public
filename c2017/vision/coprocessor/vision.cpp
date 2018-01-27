#include "c2017/vision/coprocessor/vision.h"

#include <fcntl.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <opencv2/opencv.hpp>

#include <memory>
#include <thread>
#include <vector>

#include "muan/vision/vision.h"

#define VIDEO_OUTPUT_SCREEN 0
#define VIDEO_OUTPUT_FILE 0

namespace c2017 {
namespace vision {

// The two parts of the vision targets are the ones with the closest angle
// and distance values.
VisionInputProto CalculatePosition(
    std::vector<muan::vision::ContourProperties> targets,
    const muan::vision::Vision& vision, cv::Mat debug_image) {
  VisionInputProto retval;
  retval->set_target_found(false);
  double best_distance = 0.2;  // If calculating based on the two parts of the
  // target don't get within 20cm, it isn't the target.
  cv::Rect outline;
  for (auto target_high : targets) {
    double angle_high = vision.CalculateAngle(target_high.x);
    double distance_high =
        vision.CalculateDistance(target_high.y, kHeightUpper);
    for (auto target_low : targets) {
      if (target_low.y == target_high.y && target_low.x == target_high.x) {
        continue;
      }
      double angle_low = vision.CalculateAngle(target_low.x);
      double distance_low =
          vision.CalculateDistance(target_low.y, kHeightLower);
      double difference_angle = angle_high - angle_low;
      double difference_distance = distance_high - distance_low;
      // At small angles sin(x)=x and cos(x)=1
      double distance = std::sqrt(difference_angle * difference_angle +
                                  difference_distance * difference_distance);
      if (distance < best_distance) {
        // Calculate distance
        retval->set_target_found(true);
        retval->set_angle_to_target((angle_low + angle_high) / 2);
        retval->set_distance_to_target((distance_low + distance_high) / 2);
        // Output
        outline.x =
            (target_high.x - target_high.width / 2 + 0.5) * debug_image.cols;
        outline.y = (-target_high.y + 0.5) * debug_image.rows;
        outline.width = target_high.width * debug_image.cols;
        outline.height = (target_high.y - target_low.y) * debug_image.cols;
      }
    }
  }
  cv::rectangle(debug_image, outline, cv::Scalar(0, 0, 255));
  return retval;
}

void RunVision(int camera_index) {
  // Read from file
  VisionConstants robot_constants;
  muan::vision::VisionThresholds thresholds;

  {
    int file = open(
        "c2017/vision/coprocessor/constants/robot_constants.pb.text", O_RDONLY);
    google::protobuf::io::FileInputStream fstream(file);
    google::protobuf::TextFormat::Parse(&fstream, &robot_constants);
  }

  {
    int file =
        open("c2017/vision/coprocessor/constants/thresholds.pb.text", O_RDONLY);
    google::protobuf::io::FileInputStream fstream(file);
    google::protobuf::TextFormat::Parse(&fstream, &thresholds);
  }

#if VIDEO_OUTPUT_FILE
  CvVideoWriter* writer =
      cvCreateVideoWriter("output.avi", -1, 30, cv::Size(640 * 2, 480), 1);
#endif
  cv::VideoCapture cap;
  cap.open(camera_index);

  muan::vision::VisionConstants constants{
      robot_constants.x_fov(),
      robot_constants.y_fov(),
      robot_constants.x_camera_angle(),
      robot_constants.y_camera_angle(),
      0.0005,  // Target size is not different per robot
      0.04};

  muan::vision::Vision vision{thresholds, constants};
  cv::Mat raw;
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  while (true) {
    cap >> raw;
    // Don't detect the bottom 1/3rd of the image. The target isn't there,
    // and there is often more noise and reflective robot parts.
    cv::rectangle(raw, cv::Point(0, raw.rows),
                  cv::Point(raw.cols, raw.rows * 2 / 3), cv::Scalar(0, 0, 0),
                  -1, 8, 0);
    cv::Mat image_canvas = raw.clone();
    auto targets = vision.Update(raw, image_canvas);

    VisionInputProto position;
    if (targets.size() > 1) {
      position = CalculatePosition(targets, vision, image_canvas);
    } else {
      position->set_target_found(false);
    }

    vision_queue.WriteMessage(position);

#if VIDEO_OUTPUT_FILE || VIDEO_OUTPUT_SCREEN
    cv::Mat splitscreen(image_canvas.rows, image_canvas.cols * 2, CV_8UC3);
    image_canvas.copyTo(
        splitscreen(cv::Rect(0, 0, image_canvas.cols, image_canvas.rows)));
    raw.copyTo(splitscreen(
        cv::Rect(image_canvas.cols, 0, image_canvas.cols, image_canvas.rows)));
    IplImage c_image = splitscreen;
#endif
#if VIDEO_OUTPUT_SCREEN
    cvShowImage("vision", &c_image);
    cv::waitKey(1);
#endif
#if VIDEO_OUTPUT_FILE
    cvWriteFrame(writer, &c_image);
#endif
  }
}

}  // namespace vision
}  // namespace c2017
