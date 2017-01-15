#include "climber.h"

using namespace c2017;
using namespace climber;

namespace c2017 {

namespace climber {
Climber::Climber() {

    }

    void Climber::Update (ClimberGoalProto goal, ClimberInputProto input) {
        Voltage voltage_; //Check if Voltage is real, if not change to double 
        bool hit_top_;
        bool is_climbing_;
        bool to_climb = goal->climbing();
    
        if(to_climb){
            is_climbing_ = true;
            if (input->current() < 134) { // TODO tune current spike #
                hit_top_ = false;
                voltage_ = 12.0 * V; // TODO determine the direction for the climbing motors/shooter motors
                if (input->current() > 134) { // TODO tune the current spike number ( currently its 134)
                    voltage_ = 0.0 * V;
                    hit_top_ = true;
                    is_climbing_ = true
                } else {
                    voltage = 12.0 * V;
                }
            }
        } else {
            is_climbing_ = false;
        }
      output_->set_voltage(voltage_);
      status_->set_climbing(is_climbing);
      status_->set_top(hit_top_);
    }  // Update
    ClimberOutputProto Climber::Output() { return output_; } 
    ClimberStatusProto Climber::Status() { return status_; }

}  // climber

}  // c2017
