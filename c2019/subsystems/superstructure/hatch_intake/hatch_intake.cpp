#include "c2019/subsystems/superstructure/hatch_intake/hatch_intake.h"

namespace c2019 {
namespace hatch_intake {

void HatchIntake::SetGoal(const HatchIntakeGoalProto& goal) {
  goal_ = goal;
  switch (goal_->goal()) {
    case NONE:
      break;
    case INTAKE:
      state_ = (INTAKING);
      break;
    case HOLD:
      state_ = (CARRYING);
      break;
    case SCORE:
      counter_ = 0;
      state_ = (OUTTAKING);
      break;
  }
}

void HatchIntake::Update(const HatchIntakeInputProto& input,
                         HatchIntakeOutputProto* output,
                         HatchIntakeStatusProto* status, bool outputs_enabled) {
  bool backplate = false;
  bool flutes = false;

  switch (state_) {
    case IDLE:
      flutes = false;
      backplate = false;
      break;
    case INTAKING:
      flutes = true;
      backplate = false;
      if (input->hatch_proxy()) {
        backplate = true;
        state_ = (CARRYING);
      }
      break;
    case CARRYING:
      flutes = true;
      backplate = true;
      break;
    case OUTTAKING:
      flutes = false;
      backplate = true;
      counter_++;
      if (!input->hatch_proxy() && counter_ > kScoreTicks) {
        counter_ = 0;
        flutes = false;
        backplate = false;
        state_ = (IDLE);
      }
      break;
  }
  if (outputs_enabled) {
    (*output)->set_flute_solenoid(flutes);
    (*output)->set_backplate_solenoid(backplate);
  } else {
    (*output)->set_flute_solenoid(false);
    (*output)->set_backplate_solenoid(false);
  }

  (*status)->set_has_hatch(input->hatch_proxy());
  (*status)->set_state(state_);
}

}  // namespace hatch_intake
}  // namespace c2019
