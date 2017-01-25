#include <limits>
#include "c2017/subsystems/superstructure/climber/climber.h"

namespace c2017 {

namespace climber {
Climber::Climber() :
  at_top_(false),
  is_climbing_(false),
  last_position_(0),
  status_queue_(QueueManager::GetInstance().climber_status_queue()),
  climber_position_watcher_(1/0.001, 0.25, std::numeric_limits<int>::max(), 0.005),
  climber_current_watcher_(100, 0.1, std::numeric_limits<int>::max(), 0.005),
  on_rope_(false) {}

void Climber::SetGoal(const ClimberGoalProto& goal) {
    to_climb_ = goal->climbing();
}

ClimberOutputProto Climber::Update(const ClimberInputProto& input,
                                   const muan::wpilib::
                                   DriverStationProto& ds_status) {
  RobotMode robot_state = ds_status->mode();
  double voltage_;

  ClimberStatusProto status;
  ClimberOutputProto output;
  if (robot_state == RobotMode::TELEOP) {
    if (to_climb_) {
      if (on_rope_) {
        voltage_ = climber_position_watcher_.Update(12, 1/(input->position() - last_position_));
      } else {
        voltage_ = climber_current_watcher_.Update(12, input->current());
        on_rope_ = climber_current_watcher_.is_stalled();
      }
      is_climbing_ = true;
      at_top_ = climber_position_watcher_.is_stalled();

    } else {
      is_climbing_ = false;
      voltage_ = 0.0;
    }

  } else {
    voltage_ = 0.0;
    climber_position_watcher_.Reset();
    climber_current_watcher_.Reset();
  }
  last_position_ = input->position();
  output->set_voltage(voltage_);
  status->set_currently_climbing(is_climbing_);
  status->set_hit_top(at_top_);
  status->set_on_rope(on_rope_);

  status_queue_.WriteMessage(status);

  return output;
}
void Climber::Reset() {
  climber_position_watcher_.Reset();
  climber_current_watcher_.Reset();
  at_top_ = false;
  is_climbing_ = false;
  last_position_ = 0;
  on_rope_ = false;
}
}  // namespace climber

}  // namespace c2017
