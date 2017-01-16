#include "climber.h"

namespace c2017 {

namespace climber {
Climber::Climber() : voltage_ (0.0), at_top_(false),  is_climbing_ (false), last_position_ (0) {}
void Climber::SetGoal(const ClimberGoalProto& goal) {
  goal_ = goal;
}
ClimberOutputProto Climber::Update(const ClimberInputProto& input, const muan::wpilib::DriverStationProto& ds_status) {
  RobotMode robot_state = ds_status->mode();
  
  if(robot_state == RobotMode::TELEOP){
    bool to_climb = goal_->climbing();
    
    if(to_climb){
      is_climbing_ = true;
      
      if ((input->position() - last_position_) > 5000 && at_top_ == false) { // TODO tune rate number
        voltage_ = 12.0; // TODO determine the direction for the climbing motors/shooter motors
      
      } else { 
          voltage_ = 0.0;
          at_top_ = true;
        }
    
    } else {
        is_climbing_ = false;
      }
      
    output_->set_voltage(voltage_);
    status_->set_currently_climbing(is_climbing_);
    status_->set_hit_top(at_top_);
  }
  last_position_ = input->position();
  return output_;
}  // Update

ClimberStatusProto Climber::Status() { return status_; }

}  // climber

}  // c2017
