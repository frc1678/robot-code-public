#include <WPILib.h>
#include "generic_robot/wpilib_update/main.h"
#include "subsystem_runner/subsystem_runner.h"

class WpilibRobot : public IterativeRobot {
 public:
  WpilibRobot() = default;

  void TeleopPeriodic() override { main_.Update(); }

 private:
  generic_robot::SubsystemRunner subsystem_runner_;
  std::thread subsystem_thread{std::ref(subsystem_runner_)};

  generic_robot::wpilib_update::Main main_;
};

START_ROBOT_CLASS(WpilibRobot);
