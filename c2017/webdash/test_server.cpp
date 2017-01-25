#include <thread>
#include "c2017/webdash/server.h"

int main()
{
  c2017::webdash::WebDashRunner runner;
  std::thread runner_thread{std::ref(runner)};
  while (true) {
    c2017::webdash::WebDashController::sleep(10);
  }
}
