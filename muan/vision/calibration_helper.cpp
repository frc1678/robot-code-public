#include <opencv2/opencv.hpp>
#include <fcntl.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "gflags/gflags.h"
#include "muan/vision/config.pb.h"
#include "muan/vision/vision.h"

DEFINE_string(mode, "capture",
              "The mode, either capture or tune, to run in. Capture mode "
              "captures a video for later use, while "
              "tune allows you to tune vision parameters off of a previously "
              "captured video.");
DEFINE_string(
    filename, "/tmp/capture.avi",
    "The filename to write video to/read video from depending on mode.");
DEFINE_string(
    params_file, "/tmp/thresholds.pb.text",
    "The filename to which params should be written, if in tuning mode.");
DEFINE_int32(camera_index, 0,
             "The ID of the camera to use to capture video, in capture mode.");

constexpr int kEscapeKey = 27;

void Capture(const std::string& filename, int camera_index = -1) {
  cv::VideoCapture camera(camera_index);
  if (!camera.isOpened()) {
    std::cout << "Could not open camera " << camera_index << std::endl;
    LOG(ERROR, "Couldn't open camera");
    return;
  }

  camera.set(CV_CAP_PROP_BRIGHTNESS, 0);  // minimum
  camera.set(CV_CAP_PROP_CONTRAST, 1);    // maximum
  camera.set(CV_CAP_PROP_SATURATION, 1);  // maximum
  std::string command = "v4l2ctrl -d /dev/video" +
                        std::to_string(camera_index) +
                        " -l c2017/vision/coprocessor/camera_params";
  std::cout << command << std::endl;
  system(command.c_str());

  int ex = CV_FOURCC('P', 'I', 'M', '1');
  cv::Size video_size = cv::Size(camera.get(CV_CAP_PROP_FRAME_WIDTH),
                                 camera.get(CV_CAP_PROP_FRAME_HEIGHT));
  cv::VideoWriter writer;
  if (!writer.open(filename.c_str(), ex, 30, video_size, true)) {
    std::cout << "Could not open file " << filename << " for writing"
              << std::endl;
    LOG(ERROR, "Couldn't open" << filename << " to write in");
    return;
  }

  cv::Mat frame;
  cv::namedWindow("Capture");

  while (cv::waitKey(10) != kEscapeKey) {
    camera >> frame;
    cv::imshow("Capture", frame);
    writer.write(frame);
  }

  camera.release();
  writer.release();
}

void Tune(const std::string& video_filename,
          const std::string& output_filename) {
  cv::VideoCapture video(video_filename);
  if (!video.isOpened()) {
    std::cout << "Could not open video: " << video_filename << std::endl;
    return;
  }

  muan::vision::VisionThresholds thresholds;
  {
    int file = open(output_filename.c_str(), O_RDONLY);
    google::protobuf::io::FileInputStream fstream(file);
    google::protobuf::TextFormat::Parse(&fstream, &thresholds);
    close(file);
  }

  int colorspace_code = muan::vision::ConversionCode(
      muan::vision::VisionThresholds::Bgr, thresholds.space());
  switch (thresholds.space()) {
    case muan::vision::VisionThresholds::Rgb:
      std::cout << "Tuning in RGB" << std::endl;
      break;
    case muan::vision::VisionThresholds::Hsv:
      std::cout << "Tuning in HSV" << std::endl;
      break;
    case muan::vision::VisionThresholds::Bgr:
      std::cout << "Tuning in BGR" << std::endl;
      break;
    default:
      break;
  }

  int a_min = thresholds.a_low();
  int b_min = thresholds.b_low();
  int c_min = thresholds.c_low();
  int a_max = thresholds.a_high();
  int b_max = thresholds.b_high();
  int c_max = thresholds.c_high();

  cv::namedWindow("Parameters");
  cv::createTrackbar("A min", "Parameters", &a_min, 255);
  cv::createTrackbar("B min", "Parameters", &b_min, 255);
  cv::createTrackbar("C min", "Parameters", &c_min, 255);
  cv::createTrackbar("A max", "Parameters", &a_max, 255);
  cv::createTrackbar("B max", "Parameters", &b_max, 255);
  cv::createTrackbar("C max", "Parameters", &c_max, 255);

  cv::namedWindow("Input");
  cv::namedWindow("Output");
  cv::Mat frame;
  while (cv::waitKey(10) != kEscapeKey) {
    video >> frame;
    if (frame.size[0] <= 0) {
      video.set(CV_CAP_PROP_POS_AVI_RATIO, 0);
      continue;
    }

    // Threshold
    cv::imshow("Input", frame);
    cv::cvtColor(frame, frame, colorspace_code);
    cv::inRange(frame, cv::Scalar(a_min, b_min, c_min),
                cv::Scalar(a_max, b_max, c_max), frame);
    cv::imshow("Output", frame);
  }

  {
    std::ofstream params(output_filename);
    thresholds.set_a_low(a_min);
    thresholds.set_b_low(b_min);
    thresholds.set_c_low(c_min);
    thresholds.set_a_high(a_max);
    thresholds.set_b_high(b_max);
    thresholds.set_c_high(c_max);

    std::string str;
    google::protobuf::TextFormat::PrintToString(thresholds, &str);
    params << str << "\n";

    std::cout << output_filename << "\n" << str << std::endl;

    params.flush();
  }
}

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  if (FLAGS_mode == "tune") {
    Tune(FLAGS_filename, FLAGS_params_file);
  } else if (FLAGS_mode == "capture") {
    Capture(FLAGS_filename, FLAGS_camera_index);
  } else {
    std::cout << "mode must be either \"tune\" or \"capture\"." << std::endl;
  }
}
