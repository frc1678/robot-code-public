#include <WPILib.h>
#include "generic_robot/citrus_robot/main.h"
#include "subsystem_runner/subsystem_runner.h"

class WpilibRobot : public IterativeRobot {
 public:
  WpilibRobot() = default;

  void TeleopPeriodic() override { citrus_robot_.Update(); }

 private:
  generic_robot::SubsystemRunner subsystem_runner_;
  std::thread subsystem_thread{std::ref(subsystem_runner_)};

  generic_robot::citrus_robot::CitrusRobot citrus_robot_;
};

START_ROBOT_CLASS(WpilibRobot);
