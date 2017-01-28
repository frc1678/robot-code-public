#include "c2017/vision/robot/reader.h"

namespace c2017 {
namespace vision {

VisionReader::VisionReader() {}

void VisionReader::operator()() {
  aos::time::PhasedLoop phased_loop(std::chrono::milliseconds(20));
  // aos::SetCurrentThreadRealtimePriority(10); // TODO: what is low priority?
  aos::SetCurrentThreadName("VisionReader");

  aos::vision::RXUdpSocket read_socket(1678);
  void* buffer = malloc(1024);
  running_ = true;

  while (running_) {
    std::cout << "Running!" << std::endl;
    read_socket.Recv(buffer, 1024);
    c2017::vision::VisionInputProto position;
    position->ParseFromArray(buffer, 1024);

    std::cout << "target found: " << position->target_found() << std::endl;
    if (position->has_distance_to_target()) {
      std::cout << "distance to target: " << position->distance_to_target() << std::endl;
    }
    if (position->has_angle_to_target()) {
      std::cout << "angle target: " << position->angle_to_target() << std::endl;
    }

    phased_loop.SleepUntilNext();
  }
}

void VisionReader::Stop() { running_ = false; }
}  // namespace vision
}  // namespace c2017
