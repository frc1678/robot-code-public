#include "c2017/vision/coprocessor/sender.h"
#include "third_party/aos/common/util/phased_loop.h"

namespace c2017 {
namespace vision {

void RunSender(const char* target_ip) {
  aos::vision::TXUdpSocket sender_socket(target_ip, 1678);
  aos::time::PhasedLoop phased_loop(std::chrono::milliseconds(50));
  auto queue_reader = vision_queue.MakeReader();
  char buffer[1024];
  while (true) {
    phased_loop.SleepUntilNext();
    auto message = queue_reader.ReadLastMessage();
    if (message) {
      message.value()->SerializeToArray(buffer, 1024);
    }
    sender_socket.Send(buffer, 1024);
  }
}

c2017::vision::VisionInputQueue vision_queue;

}  // namespace vision
}  // namespace c2017
