#include <string>
#include <thread>
#include <vector>
#include "muan/teleop/queue_types.h"
#include "muan/webdash/server.h"

int main() {
  muan::teleop::JoystickStatusQueue joystick_status_queue;

  muan::webdash::WebDashRunner runner;
  const std::vector<std::string> auto_list = {"test_auto", "teacher_assistant", "torque_anger"};
  runner.AddQueue("joystick_status", &joystick_status_queue);
  runner.AddAutos(&auto_list);
  std::thread webdash_test_thread{std::ref(runner)};
  webdash_test_thread.detach();

  int t = 0;
  while (true) {
    muan::teleop::JoystickStatusProto joystick_status_proto;
    joystick_status_proto->set_button1(t % 2);
    joystick_status_proto->set_button2(t % 5 <= 2);
    t++;
    joystick_status_queue.WriteMessage(joystick_status_proto);
  }
}
