#include "sender.h"

namespace c2017 {
namespace vision {

void RunSender() {
  aos::vision::TXUdpSocket sender_socket("10.16.78.2", 1678);
  auto queue_reader = vision_queue.MakeReader();
  void* buffer = malloc(1024);
  while(true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    auto message = queue_reader.ReadLastMessage();
    if (message) {
      message.value()->SerializeToArray(buffer, 1024);
    }
    std::cout << "Running!" << std::endl;
    sender_socket.Send(buffer, 1024);
  }
}

muan::vision::VisionPositionQueue vision_queue;

}
}
