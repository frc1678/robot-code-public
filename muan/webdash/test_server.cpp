#include <string>
#include <thread>
#include <vector>
#include "muan/teleop/queue_types.h"
#include "muan/webdash/server.h"
#include "c2017/subsystems/superstructure/shooter/queue_types.h"

int main() {
  muan::teleop::JoystickStatusQueue joystick_status_queue;
  muan::teleop::JoystickStatusProto joystick_status_proto;
  joystick_status_queue.WriteMessage(joystick_status_proto);

  c2017::shooter::ShooterStatusQueue shooter_status_queue;
  c2017::shooter::ShooterStatusProto shooter_status_proto;
  shooter_status_queue.WriteMessage(shooter_status_proto);

  muan::webdash::WebDashRunner runner(muan::webdash::ROBORIO);
  const std::vector<std::string> auto_list = {"test_auto", "teacher_assistant", "torque_anger"};
  runner.AddQueue("joystick_status", &joystick_status_queue);
  runner.AddQueue("shooter_status", &shooter_status_queue);
  runner.AddAutos(auto_list);
  std::string display_object =
    "{"
    "  \"widgets\": ["
    "     {"
    "       \"name\": \"Button 1\","
    "       \"type\": \"boolean\","
    "       \"source\": [\"joystick_status\", \"button1\"],"
    "       \"coordinates\": [1, 1],"
    "       \"colors\": {"
    "         \"if_true\": \"#00ff00\","
    "         \"if_false\": \"#ff0000\""
    "       }"
    "     },"
    "     {"
    "       \"name\": \"Shooter Speed\","
    "       \"type\": \"number\","
    "       \"source\": [\"shooter_status\", \"observedVelocity\"],"
    "       \"coordinates\": [0, 0],"
    "       \"min\": 0,"
    "       \"max\": 350,"
    "       \"goal\": [\"shooter_status\", \"observedVelocity\"],"
    "       \"colors\": {"
    "         \"min\": \"#000000\","
    "         \"max\": \"#000000\","
    "         \"goal\": \"#000000\""
    "       }"
    "     }"
    "  ],"
    "  \"settings\": {"
    "    \"size\": [3, 3]"
    "  }"
    "}";
  runner.DisplayObjectMaker(display_object);

  shooter_status_proto->set_observed_velocity(100);
  shooter_status_queue.WriteMessage(shooter_status_proto);

  joystick_status_proto->set_button1(true);
  joystick_status_queue.WriteMessage(joystick_status_proto);

  std::thread webdash_thread{std::ref(runner)};
  webdash_thread.detach();

  int t = 0;
  while (true) {
    muan::teleop::JoystickStatusProto joystick_status_proto;
    joystick_status_proto->set_button1(t % 2);
    shooter_status_proto->set_observed_velocity(t * 5);
    sleep(1);
    t++;
    joystick_status_queue.WriteMessage(joystick_status_proto);
    shooter_status_queue.WriteMessage(shooter_status_proto);
  }
}
