#include <thread>
#include "c2017/vision/robot/reader.h"

int main() {
  c2017::vision::VisionReader reader;
  std::thread reader_thread(reader);
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  return 0;
}
