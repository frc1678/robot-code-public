#include "c2019/subsystems/superstructure/cargo_intake/cargo_intake.h"

namespace c2019 {
namespace cargo_intake {

CargoIntake::CargoIntake() {}

void CargoIntake::Update(const CargoIntakeInputProto& input,
                         CargoIntakeOutputProto* output,
                         CargoIntakeStatusProto* status, bool outputs_enabled) {
  double roller_voltage = 0;

  if (outputs_enabled) {
    switch (state_) {
      case IDLE:
        if (input->cargo_proxy()) {
          roller_voltage = 4;
        } else {
          roller_voltage = 0;
        }
        break;
      case INTAKING:
        roller_voltage = 12;
        if (input->cargo_proxy()) {
          state_ = HOLDING;
          pickup_counter_ = 0;
        }
        break;
      case PICKING_UP:
        roller_voltage = 12;
        pickup_counter_++;
        if (pickup_counter_ > kPickupTicks) {
          state_ = HOLDING;
          pickup_counter_ = 0;
        }
      case HOLDING:
        if (input->cargo_proxy()) {
          roller_voltage = 4;
        } else {
          roller_voltage = 0;
        }
        break;
      case OUTTAKING:
        roller_voltage = -10;
        outtake_counter_++;
        if (outtake_counter_ > kOuttakeTicks) {
          state_ = IDLE;
          outtake_counter_ = 0;
        }
        break;
    }
  } else {
    roller_voltage = 0;
  }

  (*output)->set_roller_voltage(roller_voltage);
  (*status)->set_state(state_);
  prev_state_ = state_;
  (*status)->set_has_cargo(input->cargo_proxy());
}

void CargoIntake::SetGoal(const CargoIntakeGoalProto& goal) {
  switch (goal->goal()) {
    case NONE:
      state_ = IDLE;
      break;
    case INTAKE:
      state_ = INTAKING;
      break;
    case STOP_INTAKE:
      state_ = HOLDING;
      break;
    case OUTTAKE:
      outtake_counter_ = 0;
      state_ = OUTTAKING;
      break;
  }
}

}  // namespace cargo_intake
}  // namespace c2019
