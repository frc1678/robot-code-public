#include "generic_robot/citrus_robot/main.h"
#include <WPILib.h>
#include "generic_robot/subsystems/subsystem_runner.h"
#include "gflags/gflags.h"

class WpilibRobot : public IterativeRobot {
 public:
  WpilibRobot() { generic_robot::QueueManager::GetInstance()->StartLogging(); }

  void RobotInit() override {}

  void RobotPeriodic() override {}

 private:
  generic_robot::SubsystemRunner subsystem_runner_;
  generic_robot::citrus_robot::CitrusRobot main_;

  // Other threads such as reading from network may also be necessary
  std::thread subsystem_thread{std::ref(subsystem_runner_)};
  std::thread citrus_robot_thread{std::ref(main_)};
};

int main(int argc, char **argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  if (!HAL_Initialize(500, 0)) {
    std::cerr << "FATAL ERROR: HAL could not be initialized" << std::endl;
    return -1;
  }
  HAL_Report(HALUsageReporting::kResourceType_Language,
             HALUsageReporting::kLanguage_CPlusPlus);
  static WpilibRobot robot;
  std::printf("Robot program starting\n");
  robot.StartCompetition();
}
