#include "reader.h"
#include <thread>

int main() {
  std::thread reader_thread(c2017::vision::RunReader);
  while(true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  return 0;
}
