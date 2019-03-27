#include "c2019/subsystems/superstructure/hatch_intake/hatch_intake.h"

namespace c2019 {
namespace hatch_intake {

void HatchIntake::SetGoal(const HatchIntakeGoalProto& goal) {
  goal_ = goal;
  switch (goal_->goal()) {
    case NONE:
      break;
    case INTAKE:
      state_ = INTAKING;
      break;
    case HOLD:
      state_ = CARRYING;
      break;
    case SCORE:
      counter_ = 0;
      state_ = OUTTAKING;
      break;
    case PREP_SCORE:
      state_ = PREPPING_SCORE;
      break;
    case HANDOFF:
      state_ = HANDOFF_INTAKING;
      break;
  }
  force_backplate_ = goal->force_backplate();
}

void HatchIntake::Update(const HatchIntakeInputProto& input,
                         HatchIntakeOutputProto* output,
                         HatchIntakeStatusProto* status, bool outputs_enabled) {
  bool backplate = false;  // not out
  bool flutes = false;     // not out

  switch (state_) {
    case IDLE:
      flutes = true;
      backplate = false;
      if (input->hatch_proxy()) {
        backplate = true;
        state_ = (CARRYING);
      }
      break;
    case INTAKING:
      flutes = true;
      backplate = true;
      if (input->hatch_proxy()) {
        backplate = false;
        state_ = (CARRYING);
      }
      break;
    case CARRYING:
      flutes = true;
      backplate = false;
      break;
    case OUTTAKING:
      flutes = false;
      backplate = counter_ < 10;
      counter_++;
      if (counter_ > kScoreTicks) {
        counter_ = 0;
        flutes = false;
        state_ = (IDLE);
      }
      break;
    case PREPPING_SCORE:
      flutes = true;
      backplate = true;
      break;
    case HANDOFF_INTAKING:
      flutes = false;
      backplate = false;
      if (input->hatch_proxy()) {
        flutes = true;
        backplate = false;
        state_ = (CARRYING);
      }
      break;
  }
  if (outputs_enabled) {
    (*output)->set_flute_solenoid(flutes);
    (*output)->set_backplate_solenoid(backplate);
    if (force_backplate_) {
      (*output)->set_backplate_solenoid(false);
    }
  } else {
    (*output)->set_flute_solenoid(true);
    (*output)->set_backplate_solenoid(false);
  }

  bool proxy = input->hatch_proxy();
  if (!proxy && has_hatch_) {
    counter_++;
    if (counter_ > 50) {
      has_hatch_ = false;
      counter_ = 0;
    }
  } else {
    has_hatch_ = proxy;
  }
  (*status)->set_has_hatch(has_hatch_);
  (*status)->set_state(state_);
}

}  // namespace hatch_intake
}  // namespace c2019
