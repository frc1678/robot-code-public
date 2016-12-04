#include <WPILib.h>
#include "can_wrapper.h"
#include "third_party/aos/common/time.h"
#include "third_party/aos/common/util/phased_loop.h"

class BuildTestRobot : public RobotBase {
 public:
  void StartCompetition() override {
    muan::wpilib::PdpWrapper::Queue pdp_queue;
    auto pdp_reader = pdp_queue.MakeReader();
    muan::wpilib::CanWrapper can{&pdp_queue};
    std::thread can_thread{std::ref(can)};

    while (true) {
      if (auto msg = pdp_reader.ReadMessage()) {
        printf("%f\n", (*msg)->temperature());
      }
    }
  }
};

START_ROBOT_CLASS(BuildTestRobot);
