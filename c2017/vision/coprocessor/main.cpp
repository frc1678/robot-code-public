#include "sender.h"
#include "vision.h"
#include <thread>

int main() {
  std::thread sender_thread(c2017::vision::RunSender);
  std::thread vision_thread(c2017::vision::RunVision);
  while(true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  return 0;
}
