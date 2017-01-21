#include <iostream>
#include "climber.h"
#include "c2017/queue_manager/queue_manager.h"

namespace c2017 {

namespace climber {
Climber::Climber() : at_top_(false), is_climbing_(false), last_position_(0), status_queue_(QueueManager::GetInstance().climber_status_queue()) {}

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
      is_climbing_ = true;

      if (input->position() - last_position_ > 1e-3) {  // TODO tune rate number
        voltage_ = 12.0;

      } else {
        voltage_ = 0.0;
        at_top_ = true;
      }

    } else {
      is_climbing_ = false;
      voltage_ = 0.0;
    }

  } else {
    voltage_ = 0.0;
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
