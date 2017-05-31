#include "generic_robot/citrus_robot/main.h"
#include <WPILib.h>
#include "gflags/gflags.h"
#include "generic_robot/subsystems/subsystem_runner.h"

class WpilibRobot : public IterativeRobot {
 public:
  WpilibRobot() {}

  void RobotInit() override {}

  void RobotPeriodic() override { main_.Update(); }

 private:
  generic_robot::SubsystemRunner subsystem_runner_;
  std::thread subsystem_thread{std::ref(subsystem_runner_)};

  generic_robot::citrus_robot::CitrusRobot main_;
};

int main(int argc, char **argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  if (!HAL_Initialize(0)) {
    std::cerr << "FATAL ERROR: HAL could not be initialized" << std::endl;
    return -1;
  }
  HAL_Report(HALUsageReporting::kResourceType_Language, HALUsageReporting::kLanguage_CPlusPlus);
  static WpilibRobot robot;
  std::printf("Robot program starting\n");
  robot.StartCompetition();
}
