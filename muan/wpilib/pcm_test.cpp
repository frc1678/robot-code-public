#include <WPILib.h>
#include "can_wrapper.h"
#include "third_party/aos/common/time.h"
#include "third_party/aos/common/util/phased_loop.h"

class BuildTestRobot : public RobotBase {
 public:
  void StartCompetition() override{
    muan::wpilib::PdpWrapper::Queue pdp_queue;
    muan::wpilib::CanWrapper can{&pdp_queue};
    std::thread can_thread{std::ref(can)};

    aos::time::PhasedLoop loop{aos::time::Time::InMS(10)};

    can.pcm()->CreateSolenoid(6);

    HALNetworkCommunicationObserveUserProgramStarting();

    while (true) {
      if (IsDisabled()) {
        can.pcm()->WriteSolenoid(6, false);
        HALNetworkCommunicationObserveUserProgramDisabled();
      }
      else if (IsAutonomous()) {
        can.pcm()->WriteSolenoid(6, true);
        HALNetworkCommunicationObserveUserProgramAutonomous();
      }
    }
  }
};

START_ROBOT_CLASS(BuildTestRobot);
