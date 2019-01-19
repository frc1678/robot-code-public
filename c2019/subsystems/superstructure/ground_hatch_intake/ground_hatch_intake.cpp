#include "c2019/subsystems/superstructure/ground_hatch_intake/ground_hatch_intake.h"

namespace c2019 {
namespace ground_hatch_intake {

void GroundHatchIntake::Update(bool outputs_enabled,
                               const GroundHatchIntakeInputProto& input,
                               GroundHatchIntakeOutputProto* output,
                               GroundHatchIntakeStatusProto* status) {
  double voltage = 0;
  bool snap_down = false;

  if (outputs_enabled) {
    switch (current_state_) {
      case IDLE:
        voltage = 0;
        snap_down = false;
        break;
      case INTAKING:
        voltage = kIntakeVoltage;
        snap_down = true;
        if (input->current() > kCurrentThreshold) {
          current_state_ = PICKING_UP;
        }
        break;
      case PICKING_UP:
        voltage = kIntakeVoltage;
        snap_down = true;
        counter_++;
        if (counter_ > kPickupTicks) {
          current_state_ = CARRYING;
          counter_ = 0;
        }
        break;
      case CARRYING:
        voltage = kHoldingVoltage;
        snap_down = false;
        break;
      case OUTTAKING:
        voltage = kOuttakeVoltage;
        snap_down = false;
        break;
    }
  }

  (*output)->set_roller_voltage(voltage);
  (*output)->set_snap_down(snap_down);
  (*status)->set_state(current_state_);
  (*status)->set_has_hatch(current_state_ == CARRYING);
}

void GroundHatchIntake::SetGoal(GroundHatchIntakeGoalProto goal) {
  switch (goal->goal()) {
    case NONE:
      break;
    case REQUEST_HATCH:
      current_state_ = INTAKING;
      break;
    case EJECT:
      current_state_ = OUTTAKING;
      break;
    case RISE:
      current_state_ = IDLE;
      break;
  }
}

}  // namespace ground_hatch_intake
}  // namespace c2019
