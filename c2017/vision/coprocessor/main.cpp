#include <iostream>
#include <string>
#include <thread>
#include "c2017/vision/coprocessor/sender.h"
#include "c2017/vision/coprocessor/vision.h"
#include "gflags/gflags.h"

DEFINE_string(robot_ip, "10.16.78.20", "The IP address of the roborio");
DEFINE_int32(camera_index, 1, "The index of the camera used for vision. 0 opens /dev/video0, etc.");

int main(int n, char** args) {
  gflags::ParseCommandLineFlags(&n, &args, true);
  std::thread sender_thread(c2017::vision::RunSender, FLAGS_robot_ip.c_str());
  std::thread vision_thread(c2017::vision::RunVision, FLAGS_camera_index);
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  return 0;
}
