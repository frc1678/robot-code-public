#include "reader.h"

namespace c2017 {
namespace vision {

void RunReader() {
  aos::vision::RXUdpSocket read_socket(1678);
  void* buffer = malloc(1024);
  while(true) {
    read_socket.Recv(buffer, 1024);
    muan::vision::VisionPositionProto position;
    position->ParseFromArray(buffer, 1024);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    std::cout << "target found: " << position->target_found() << std::endl;
    if (position->has_target_id()) {
      std::cout << "target id: " << position->target_id() << std::endl;
    }
    if (position->has_distance_to_target()) {
      std::cout << "distance to target: " << position->distance_to_target() << std::endl;
    }
    if (position->has_angle_to_target()) {
      std::cout << "angle target: " << position->angle_to_target() << std::endl;
    }
  }
}

}
}
