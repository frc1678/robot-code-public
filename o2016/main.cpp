#include <WPILib.h>
#include "subsystem_runner/subsystem_runner.h"

class WpilibRobot : public IterativeRobot
{
  public:
    WpilibRobot() = default;

  private:
    o2016::SubsystemRunner subsystem_runner_;
    std::thread subsystem_thread{std::ref(subsystem_runner_)};
};
START_ROBOT_CLASS(WpilibRobot);
