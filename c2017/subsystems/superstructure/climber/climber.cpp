#include "climber.h"

using namespace c2017;
using namespace climber;

namespace c2017 {

namespace climber {
Climber::Climber() {
        voltage_ = 0.0; 
        at_top_ = false;
        is_climbing_ = false;
    }
void Climber::SetGoal (const ClimberGoalProto& goal) {
  
  goal_ = goal;
}

void Climber::Update (const ClimberInputProto& input, const muan::wpilib::DriverStationProto& ds_status) {
        RobotMode robot_state = ds_status->mode();
        if(robot_state == RobotMode::TELEOP){
        bool to_climb = goal_->climbing();
        if(to_climb){
            is_climbing_ = true;
            if (input->current() < 134) { // TODO tune current spike #
                at_top_ = false;
                voltage_ = 12.0; // TODO determine the direction for the climbing motors/shooter motors
            } else { 
                voltage_ = 0.0;
                at_top_ = true;
                is_climbing_ = true;
            }
        } else {
            is_climbing_ = false;
        }
      output_->set_voltage(voltage_);
      status_->set_currently_climbing(is_climbing_);
      status_->set_hit_top(at_top_);
    }
}// Update
    ClimberOutputProto Climber::Output() { return output_; } 
    ClimberStatusProto Climber::Status() { return status_; }

}  // climber

}  // c2017
