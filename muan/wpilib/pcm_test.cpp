#include <WPILib.h>
#include "muan/wpilib/can_wrapper.h"
#include "third_party/aos/common/time.h"
#include "third_party/aos/common/util/phased_loop.h"

class BuildTestRobot : public RobotBase {
 public:
  void StartCompetition() override {
    muan::wpilib::PdpWrapper::Queue pdp_queue(200);
    muan::wpilib::CanWrapper can{&pdp_queue};
    std::thread can_thread{std::ref(can)};

    aos::time::PhasedLoop loop{std::chrono::milliseconds(10)};

    can.pcm()->CreateSolenoid(6);

    HAL_ObserveUserProgramStarting();
    // Don't write to solenoid if disabled.
    // If in autonmous, write to solenoid
    while (true) {
      if (IsDisabled()) {
        can.pcm()->WriteSolenoid(6, false);
        HAL_ObserveUserProgramDisabled();
      } else if (IsAutonomous()) {
        can.pcm()->WriteSolenoid(6, true);
        HAL_ObserveUserProgramAutonomous();
      }
    }
  }
};

START_ROBOT_CLASS(BuildTestRobot);
