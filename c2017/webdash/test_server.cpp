#include <thread>
#include "c2017/webdash/server.h"
#include "c2017/subsystems/superstructure/shooter/queue_types.h"

int main() {
  c2017::shooter::ShooterInputQueue shooter_input_queue;
  c2017::shooter::ShooterInputProto test_input_proto;
  shooter_input_queue.WriteMessage(test_input_proto);

  c2017::shooter::ShooterStatusQueue shooter_status_queue;
  c2017::shooter::ShooterStatusProto test_status_proto;
  shooter_status_queue.WriteMessage(test_status_proto);

  c2017::webdash::WebDashRunner runner;
  runner.AddQueue("shooter_input", &shooter_input_queue);
  runner.AddQueue("shooter_status", &shooter_status_queue);
  runner();
}
