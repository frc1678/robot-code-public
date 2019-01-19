#include "c2019/subsystems/superstructure/cargo_intake/cargo_intake.h"

namespace c2019 {
namespace cargo_intake {

CargoIntake::CargoIntake() {}

void CargoIntake::Update(bool outputs_enabled,
                         const CargoIntakeInputProto& input,
                         CargoIntakeOutputProto* output,
                         CargoIntakeStatusProto* status) {
  double roller_voltage = 0;

  if (outputs_enabled) {
    switch (run_intake_) {
      case Goal::INTAKE:
        roller_voltage = 12;
        break;
      case Goal::OUTTAKE:
        roller_voltage = -4;
        break;
      case Goal::IDLE:
        if (input->has_cargo()) {
          roller_voltage = -2;
        }
        break;
    }
  } else {
    roller_voltage = 0;
  }

  (*output)->set_roller_voltage(roller_voltage);
  (*status)->set_state(run_intake_);
  (*status)->set_has_cargo(input->has_cargo());
}

void CargoIntake::SetGoal(const CargoIntakeGoalProto& goal) { run_intake_ = goal->goal(); }

}  // namespace cargo_intake
}  // namespace c2019
