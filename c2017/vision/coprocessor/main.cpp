#include <iostream>
#include <string>
#include <thread>
#include "c2017/vision/coprocessor/sender.h"
#include "c2017/vision/coprocessor/vision.h"
#include "gflags/gflags.h"

DEFINE_string(robot_ip, "10.16.78.20", "The IP address of the roborio");
DEFINE_int32(camera_index, 0, "The index of the camera used for vision. 0 opens /dev/video0, etc.");

int main(int n, char** args) {
  gflags::ParseCommandLineFlags(&n, &args, true);
  std::thread sender_thread(c2017::vision::RunSender, FLAGS_robot_ip.c_str());
  c2017::vision::RunVision(FLAGS_camera_index);
  //  This line is never reached
  return 0;
}
