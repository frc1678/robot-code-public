#include "c2017/citrus_robot/main.h"
#include <WPILib.h>
#include "c2017/webdash/server.h"
#include "gflags/gflags.h"
#include "subsystems/subsystem_runner.h"

class WpilibRobot : public IterativeRobot {
 public:
  WpilibRobot() { c2017::QueueManager::GetInstance().StartLogging(); }

  void RobotInit() override {
    // Webdash is causing timing issues, likely a Mongoose issue. TODO(Kyle and Wesley) Figure this out.
    // std::thread webdash_thread{std::ref(webdash_)};
    // webdash_thread.detach();
  }

  void RobotPeriodic() override { main_.Update(); }

 private:
  c2017::SubsystemRunner subsystem_runner_;
  std::thread subsystem_thread{std::ref(subsystem_runner_)};

  c2017::webdash::WebDashRunner webdash_;

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
