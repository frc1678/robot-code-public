#ifndef FRC971_CONTROL_LOOPS_GEAR_H_
#define FRC971_CONTROL_LOOPS_GEAR_H_

#include "third_party/frc971/control_loops/drivetrain/drivetrain.pb.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

// True if the the robot might or is trying to be in high gear.
inline bool MaybeHigh(Gear g) {
  return g == Gear::kHighGear || g == Gear::kShiftingUp;
}

// True if the shifter is engaged and ready for action.
inline bool IsInGear(Gear gear) {
  return gear == Gear::kLowGear || gear == Gear::kHighGear;
}

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_GEAR_H_
