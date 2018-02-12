#include <string>
#include <thread>
#include <vector>
#include "c2017/subsystems/superstructure/shooter/queue_types.h"
#include "muan/teleop/queue_types.h"
#include "muan/webdash/server.h"
#include "muan/wpilib/queue_types.h"

int main() {
  muan::wpilib::DriverStationProto driver_station_proto;
  muan::wpilib::DriverStationQueue driver_station_queue;
  driver_station_proto->set_alliance(kRed);
  driver_station_queue.WriteMessage(driver_station_proto);

  muan::teleop::JoystickStatusQueue joystick_status_queue;
  muan::teleop::JoystickStatusProto joystick_status_proto;
  joystick_status_queue.WriteMessage(joystick_status_proto);

  c2017::shooter::ShooterStatusQueue shooter_status_queue;
  c2017::shooter::ShooterStatusProto shooter_status_proto;
  shooter_status_queue.WriteMessage(shooter_status_proto);

  muan::wpilib::DriverStationQueue driver_station_status_queue;
  muan::wpilib::DriverStationProto driver_station_status_proto;
  driver_station_status_proto->set_alliance(Alliance::kRed);
  driver_station_status_queue.WriteMessage(driver_station_status_proto);

  muan::wpilib::GameSpecificStringQueue gss_queue;
  muan::wpilib::GameSpecificStringProto gss_proto;
  gss_proto->set_code("RRL");
  gss_queue.WriteMessage(gss_proto);

  muan::webdash::WebDashRunner runner(muan::webdash::ROBORIO);
  runner.AddQueue("joystick_status", &joystick_status_queue);
  runner.AddQueue("gss", &gss_queue);
  runner.AddQueue("shooter_status", &shooter_status_queue);
  runner.AddQueue("driver_station_status", &driver_station_queue);
  std::string display_object =
      "{"
      "  \"widgets\": ["
      "     {"
      "       \"name\": \"Button 1\","
      "       \"type\": \"boolean\","
      "       \"source\": [\"joystick_status\", \"button1\"],"
      "       \"coordinates\": [1, 1],"
      "       \"should-title\": true,"
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
      "       \"should-title\": true,"
      "       \"min\": 0,"
      "       \"max\": 350,"
      "       \"goal\": [\"shooter_status\", \"observedVelocity\"],"
      "       \"colors\": {"
      "         \"min\": \"#000000\","
      "         \"max\": \"#000000\","
      "         \"goal\": \"#000000\""
      "       }"
      "     },"
      "     {"
      "       \"name\": \"Encouragment\","
      "       \"type\": \"encouragement\","
      "       \"coordinates\": [1, 0]"
      "     },"
      "     {"
      "       \"name\": \"Game Layout\","
      "       \"type\": \"gss\","
      "       \"source\": [\"gss\", \"code\"],"
      "       \"coordinates\": [0, 2],"
      "       \"should-title\": false,"
      "       \"code\": ["
      "          \"if (!document.getElementById(\\\"gss_box_1\\\")) {\","
      "          \"  for (var i = 6; i > 0; i--) {\","
      "          \"    document.getElementById(coords).innerHTML += \\\"<div "
      "id=gss_box_\\\" + i + \","
      "          \"                                                 \\\" "
      "style=width:\\\" + \","
      "          \"                                                 "
      "box_width/2 + \","
      "          \"                                                 "
      "\\\"px;height:\\\" + \","
      "          \"                                                 "
      "box_height/3 + \","
      "          \"                                                 "
      "\\\"px></div>\\\";\","
      "          \"  }\","
      "          \"}\","
      "          \"if (source.charAt(2) == 'L') {\","
      "          \"  "
      "document.getElementById(\\\"gss_box_5\\\").style.backgroundColor = \","
      "          \"                                                            "
      "        alliance_color;\","
      "          \"  "
      "document.getElementById(\\\"gss_box_6\\\").style.backgroundColor = \","
      "          \"                                                            "
      "        other_alliance_color;\","
      "          \"} else if (source.charAt(2) == 'R') {\","
      "          \"  "
      "document.getElementById(\\\"gss_box_5\\\").style.backgroundColor = \","
      "          \"                                                            "
      "        other_alliance_color;\","
      "          \"  "
      "document.getElementById(\\\"gss_box_6\\\").style.backgroundColor = \","
      "          \"                                                            "
      "        alliance_color;\","
      "          \"}\","
      "          \"if (source.charAt(1) == 'L') {\","
      "          \"  "
      "document.getElementById(\\\"gss_box_3\\\").style.backgroundColor = \","
      "          \"                                                            "
      "        alliance_color;\","
      "          \"  "
      "document.getElementById(\\\"gss_box_4\\\").style.backgroundColor = \","
      "          \"                                                            "
      "        other_alliance_color;\","
      "          \"} else if (source.charAt(1) == 'R') {\","
      "          \"  "
      "document.getElementById(\\\"gss_box_3\\\").style.backgroundColor = \","
      "          \"                                                            "
      "        other_alliance_color;\","
      "          \"  "
      "document.getElementById(\\\"gss_box_4\\\").style.backgroundColor = \","
      "          \"                                                            "
      "        alliance_color;\","
      "          \"}\","
      "          \"if (source.charAt(0) == 'L') {\","
      "          \"  "
      "document.getElementById(\\\"gss_box_1\\\").style.backgroundColor = \","
      "          \"                                                            "
      "        alliance_color;\","
      "          \"  "
      "document.getElementById(\\\"gss_box_2\\\").style.backgroundColor = \","
      "          \"                                                            "
      "        other_alliance_color;\","
      "          \"} else if (source.charAt(0) == 'R') {\","
      "          \"  "
      "document.getElementById(\\\"gss_box_1\\\").style.backgroundColor = \","
      "          \"                                                            "
      "        other_alliance_color;\","
      "          \"  "
      "document.getElementById(\\\"gss_box_2\\\").style.backgroundColor = \","
      "          \"                                                            "
      "        alliance_color;\","
      "          \"}\""
      "       ]"
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
