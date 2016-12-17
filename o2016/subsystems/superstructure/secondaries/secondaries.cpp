#include "secondaries.h"

namespace o2016 {
namespace secondaries {

SecondariesOutputProto Secondaries::Update(SecondariesGoalProto goal) {
  SecondariesOutputProto output;

  double voltage = 0;

  if (goal->direction() == Direction::FORWARD) {
    voltage = 12;
  } else if (goal->direction() == Direction::REVERSE) {
    voltage = -12;
  } else {
    voltage = 0;
  }

  output->set_voltage(voltage);
  output->set_is_down(goal->position() == Position::DOWN);

  return output;
}

}  // o2016
}  // secondaries
