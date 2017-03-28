#include "c2017/citrus_robot/main.h"
#include <WPILib.h>
#include "gflags/gflags.h"
#include "subsystems/subsystem_runner.h"
#include "vision/robot/reader.h"

class WpilibRobot : public IterativeRobot {
 public:
  WpilibRobot() { c2017::QueueManager::GetInstance().StartLogging(); }

  void RobotInit() override {}

  void RobotPeriodic() override { main_.Update(); }

 private:
  c2017::SubsystemRunner subsystem_runner_;
  c2017::vision::VisionReader vision_reader_;
  std::thread subsystem_thread{std::ref(subsystem_runner_)};
  std::thread vision_thread{std::ref(vision_reader_)};

  c2017::citrus_robot::CitrusRobot main_;
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
