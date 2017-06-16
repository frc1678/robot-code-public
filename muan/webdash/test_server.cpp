#include <string>
#include <thread>
#include <vector>
#include "muan/teleop/queue_types.h"
#include "muan/webdash/server.h"

int main() {
  muan::teleop::JoystickStatusQueue joystick_status_queue;
  muan::teleop::JoystickStatusProto joystick_status_proto;
  joystick_status_queue.WriteMessage(joystick_status_proto);

  muan::webdash::WebDashRunner runner;
  const std::vector<std::string> auto_list = {"test_auto", "teacher_assistant", "torque_anger"};
  runner.AddQueue("joystick_status", &joystick_status_queue);
  runner.AddAutos(&auto_list);
  runner();
}
