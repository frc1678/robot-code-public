#include <WPILib.h>
#include "c2017/citrus_robot/main.h"
#include "c2017/webdash/server.h"
#include "subsystems/subsystem_runner.h"

class WpilibRobot : public IterativeRobot {
 public:
  WpilibRobot() { c2017::QueueManager::GetInstance().StartLogging(); }

  void TeleopPeriodic() override { main_.Update(); }

  void AutonomousPeriodic() override { main_.Update(); }

 private:
  c2017::SubsystemRunner subsystem_runner_;
  std::thread subsystem_thread{std::ref(subsystem_runner_)};

  c2017::webdash::WebDashRunner runner;
  std::thread runner_thread{std::ref(runner)};

  c2017::citrus_robot::CitrusRobot main_;
};

START_ROBOT_CLASS(WpilibRobot);
