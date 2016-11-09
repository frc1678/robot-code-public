#include "o2016/teleop/teleop.h"
#include "subsystem_runner/subsystem_runner.h"
#include <WPILib.h>

class WpilibRobot : public IterativeRobot {
 public:
  WpilibRobot() = default;

  void TeleopPeriodic() override { teleop_.Update(); }

 private:
  o2016::SubsystemRunner subsystem_runner_;
  std::thread subsystem_thread{std::ref(subsystem_runner_)};

  o2016::teleop::Teleop teleop_;
};

START_ROBOT_CLASS(WpilibRobot);
