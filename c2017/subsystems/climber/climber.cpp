#include "climber.h"

using namespace c2017;
using namespace climber;

namespace c2017 {

namespace climber {

ClimberOutputProto Climber::Update (ClimberGoalProto goal, ClimberInputProto input) {
  if (goal) {
        bool to_climb = goal.climbing();
    if(to_climb == true){
      
      if (at_top == true) {
        output_.set_voltage(0.0 * V);
      } else {
        output_.set_voltage(12.0 * V); // TODO determine the direction for the climbing motors/shooter motors
        status_.set_currently_climbing(true);
        if (input.current() > 134) { // TODO tune the current spike number ( currently its 134)
          output_.set_voltage(0.0 * V);
          at_top = true;
        }
      } else {
        output_.set_voltage(0.0 * V);
        status_.set_currently_climbing(false);
        }
  }
  ClimberOutputProto Climber::output() {return output_;} 
  ClimberStatusProto Climber::status() {return status_; }}
}  // Update

}  // climber

}  // c2017
