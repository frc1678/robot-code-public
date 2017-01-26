#include "magazine.h"

namespace c2017 {

namespace magazine { 

MagazineOutputProto Magazine::Update(MagazineInputProto input) {
  magazine_extended = true;
  
  switch (ConveyorGoalState) {
    case IDLE:
      conveyor_voltage = 0;
      break;
    
    case FORWARD:
      conveyor_voltage = 12;
      break;
    
    case BACKWARD:
      conveyor_voltage = -12;
      break;
  }
  switch (HPIntakeState) {
    case NONE:
      gear_intake_covered = true;
      break;

    case BALLS:
      gear_intake_covered = true;
      break;

    case GEAR:
      gear_intake_covered = false;
      break;

    case BOTH:
      if (!(has_gear)) {
        gear_intake_covered = false;
      }
      if (has_gear) {
        gear_intake_covered = true;
      }
      break;
  } 

  void Magazine::SetInput(MagazineInputProto input) {
    has_hp_gear_ = input->has_hp_gear();
    conveyor_current_ = input->conveyor_current();
  }

  void Magazine::SetGoal(MagazineGoalProto goal) {
    

} //magazine

} //c2017
