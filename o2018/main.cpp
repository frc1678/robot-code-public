#include <WPILib.h>
#include "gflags/gflags.h"
#include "muan/queues/queue_manager.h"
#include "o2018/autonomous/autonomous_runner.h"
#include "o2018/subsystems/subsystem_runner.h"
#include "o2018/teleop/teleop.h"
#include "o2018/webdash/webdash_setup.h"

class WpilibRobot : public IterativeRobot {
 public:
  WpilibRobot() {}

  void RobotInit() override {}
  void RobotPeriodic() override {}

  void SpawnThreads() {
    std::thread subsystem_thread(std::ref(subsystem_runner_));
    subsystem_thread.detach();

    std::thread teleop_thread(std::ref(teleop_base_));
    teleop_thread.detach();

    std::thread autonomous_thread(std::ref(auto_));
    autonomous_thread.detach();
  }

 private:
  o2018::subsystems::SubsystemRunner subsystem_runner_;
  o2018::teleop::TeleopBase teleop_base_;
  o2018::autonomous::AutonomousRunner auto_;
};

int main(int argc, char **argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  muan::queues::Start();
  o2018::StartWebdash();
  if (!HAL_Initialize(500, 0)) {
    std::printf("FATAL ERROR: HAL could not be initialized\n");
    return -1;
  }
  HAL_Report(HALUsageReporting::kResourceType_Language,
             HALUsageReporting::kLanguage_CPlusPlus);
  WpilibRobot robot;

  robot.SpawnThreads();

  std::printf("Robot program starting\n");
  robot.StartCompetition();
}
