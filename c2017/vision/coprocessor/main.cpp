#include <iostream>
#include <string>
#include <thread>
#include "c2017/vision/coprocessor/sender.h"
#include "c2017/vision/coprocessor/vision.h"

int main(int n, char** args) {
  if (n != 3) {
    std::cerr << "Requires RoboRIO IP as first argument and camera index as second" << std::endl;
    return -1;
  }
  c2017::vision::Vision2017 vision{std::stoi(args[2])};
  std::thread sender_thread(c2017::vision::RunSender, args[1]);
  std::thread vision_thread(vision);
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  return 0;
}
