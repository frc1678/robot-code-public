#include "c2019/subsystems/superstructure/winch/winch.h"

namespace c2019 {
namespace winch {

using muan::queues::QueueManager;
using muan::wpilib::DriverStationProto;

Winch::Winch() {}

// Set member variables using goal proto
void Winch::SetGoal(const WinchGoalProto& goal) {
  winch_ = goal->winch();
  climb_type_ = goal->climb_goal();
}

void Winch::Update(const WinchInputProto& input, WinchOutputProto* output,
                   WinchStatusProto* status, bool outputs_enabled) {
  if (outputs_enabled) {
    drop_forks_ = climb_type_ == BUDDY;  // drop forks for buddy climb
    winch_voltage_ = (winch_ && drop_forks_) ? 12. : 0.;
  } else {
    drop_forks_ = false;  // forks don't drop for solo climb
    winch_voltage_ = 0;
  }

  // Writes to status and output protos
  (*status)->set_winch_current(input->winch_current());
  (*status)->set_climb(winch_);
  (*status)->set_climb_type(climb_type_);
  (*output)->set_winch_voltage(winch_voltage_);
  (*output)->set_drop_forks(drop_forks_);
}

}  // namespace winch
}  // namespace c2019
