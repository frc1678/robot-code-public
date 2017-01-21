#include <iostream>
#include "climber.h"
#include "c2017/queue_manager/queue_manager.h"

namespace c2017 {

namespace climber {
Climber::Climber() : at_top_(false), is_climbing_(false), last_position_(0), status_queue_(QueueManager::GetInstance().climber_status_queue()), climber_watcher_(1/0.001, 0.25, std::numeric_limits<int>::max(), 0.005) {}

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
      voltage_ = climber_watcher_.Update(12, 1/(input->position() - last_position_));
      is_climbing_ = true;
      at_top_ = voltage_ < 9;

    } else {
      is_climbing_ = false;
      voltage_ = 0.0;
    }

  } else {
    voltage_ = 0.0;
    climber_watcher_.Reset();
  }
  last_position_ = input->position();
  output->set_voltage(voltage_);
  status->set_currently_climbing(is_climbing_);
  status->set_hit_top(at_top_);

  status_queue_.WriteMessage(status);

  return output;

}

}  // climber

}  // c2017
