#include "c2019/subsystems/superstructure/winch/winch.h"

namespace c2019 {
namespace winch {

using muan::queues::QueueManager;
using muan::wpilib::DriverStationProto;

Winch::Winch() {}

// Set member variables using goal proto
void Winch::SetGoal(const WinchGoalProto& goal) {
  winch_right_ = goal->winch_right();
  winch_left_ = goal->winch_left();
  climb_type_ = goal->climb_goal();
}

void Winch::Update(const WinchInputProto& input, WinchOutputProto* output,
                   WinchStatusProto* status, bool outputs_enabled) {
  if (outputs_enabled) {
    drop_forks_ = climb_type_ == BUDDY;  // drop forks for buddy climb
    right_winch_voltage_ = (winch_right_ && drop_forks_) ? 12. : 0.;
    left_winch_voltage_ = (winch_left_ && drop_forks_) ? 12. : 0.;
  } else {
    drop_forks_ = false;  // forks don't drop for solo climb
    right_winch_voltage_ = 0;
    left_winch_voltage_ = 0;
  }

  // Writes to status and output protos
  (*status)->set_winch_current(input->winch_current());
  (*status)->set_climb(winch_right_ || winch_left_);
  (*status)->set_climb_type(climb_type_);
  (*output)->set_right_winch_voltage(right_winch_voltage_);
  (*output)->set_left_winch_voltage(left_winch_voltage_);
  (*output)->set_drop_forks(drop_forks_);
}

}  // namespace winch
}  // namespace c2019
