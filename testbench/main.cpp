#include <WPILib.h>
#include "testbench/subsystems/subsystem_runner.h"
#include "testbench/teleop/teleop.h"

class TestBenchRobot : public IterativeRobot {
 public:
  TestBenchRobot() = default;

  void TeleopPeriodic() override { teleop_.Update(); }

 private:
  testbench::SubsystemRunner subsystem_runner_;
  std::thread subsystem_thread{std::ref(subsystem_runner_)};

  testbench::teleop::Teleop teleop_;
};

START_ROBOT_CLASS(TestBenchRobot);
