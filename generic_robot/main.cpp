#include <WPILib.h>
#include "generic_robot/teleop/teleop.h"
#include "subsystem_runner/subsystem_runner.h"

class WpilibRobot : public IterativeRobot {
 public:
  WpilibRobot() = default;

  void TeleopPeriodic() override { teleop_.Update(); }

 private:
  generic_robot::SubsystemRunner subsystem_runner_;
  std::thread subsystem_thread{std::ref(subsystem_runner_)};

  generic_robot::teleop::Teleop teleop_;
};

START_ROBOT_CLASS(WpilibRobot);
