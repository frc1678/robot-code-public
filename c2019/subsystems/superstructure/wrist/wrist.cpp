#include "c2019/subsystems/superstructure/wrist/wrist.h"
#include "muan/utils/math_utils.h"

namespace c2019 {
namespace wrist {

Wrist::Wrist() {}

void Wrist::SetGoal(const WristGoalProto& goal) {
  goal_ = muan::utils::Cap(goal->angle(), kMinAngle, kMaxAngle);
}

double Wrist::CalculateFeedForwards(bool has_cargo, bool has_panel,
                                    double angle) {
  double ff = kFF;
  if (has_cargo) {  // TODO(Eithne) do this correctly
    ff += kFFCargo;
  }
  if (has_panel) {  // TODO(Eithne) do this correctly
    ff += kFFHatch;
  }

  return ff * cos(angle);
}

void Wrist::Update(const WristInputProto& input, WristOutputProto* output,
                   WristStatusProto* status, bool outputs_enabled) {
  const double calibrated_encoder = input->wrist_encoder();
  (*status)->set_wrist_angle(calibrated_encoder);

  (*status)->set_is_calibrated(input->wrist_zeroed());
  (*status)->set_wrist_goal(goal_);
  (*status)->set_wrist_encoder_fault(false);
  (*status)->set_wrist_velocity(input->wrist_velocity());

  (*output)->set_wrist_setpoint_ff(1.53 * std::cos(calibrated_encoder));
  if (outputs_enabled) {
    if (input->wrist_zeroed()) {
      (*output)->set_output_type(POSITION);
      (*output)->set_wrist_setpoint(goal_);
      if (calibrated_encoder <= goal_ + 1e-2 && goal_ < 1e-2) {
        (*output)->set_output_type(OPEN_LOOP);
        (*output)->set_wrist_setpoint(0);
      }
    } else {
      (*output)->set_output_type(OPEN_LOOP);
      (*output)->set_wrist_setpoint(0);
    }
  } else {
    (*output)->set_output_type(OPEN_LOOP);
    (*output)->set_wrist_setpoint(0);
  }
}

}  // namespace wrist
}  // namespace c2019
