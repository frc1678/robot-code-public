#include "c2018/subsystems/climber/climber.h"

namespace c2018 {
namespace climber {

using muan::queues::QueueManager;
using muan::wpilib::DriverStationProto;

Climber::Climber()
    : status_queue_{QueueManager<ClimberStatusProto>::Fetch()},
      goal_reader_{QueueManager<ClimberGoalProto>::Fetch()->MakeReader()},
      input_reader_{QueueManager<ClimberInputProto>::Fetch()->MakeReader()},
      output_queue_{QueueManager<ClimberOutputProto>::Fetch()},
      ds_status_{QueueManager<DriverStationProto>::Fetch()->MakeReader()} {}

void Climber::Update() {
  bool should_climb = false;
  bool outputs_enabled = false;
  bool batter_output;
  double winch_output;

  ClimberStatusProto status;
  ClimberOutputProto output;
  ClimberGoalProto goal;
  ClimberInputProto input;

  if (!goal_reader_.ReadLastMessage(&goal)) {
    goal->set_climber_goal(NONE);
  }

  if (!input_reader_.ReadLastMessage(&input)) {
    return;
  }

  outputs_enabled = ds_status_.ReadLastMessage().value()->is_sys_active();
  // SETTING STATUS AND SHOULD_CLIMB GOAL
  if (outputs_enabled) {
    switch (goal->climber_goal()) {
      case NONE:
        should_climb = false;
        status->set_climber_state(IDLE);
        break;
      case APPROACHING:
        goal->set_put_down_batter(true);
        status->set_climber_state(APPROACH);
        break;
      case CLIMBING:
        should_climb = true;
        status->set_climber_state(CLIMB);
        break;
    }
    // SETTING STATUS
    if (winch_.has_climbed()) {
      status->set_climber_state(DONE);
    }
  } else {
    status->set_climber_state(IDLE);
  }

  // UPDATING MECHANISMS
  winch_output =
      winch_.Update(input->position(), should_climb, outputs_enabled);
  batter_output =
      batter_.Update(goal->put_down_batter(), outputs_enabled);

  // SETTING OUTPUTS
  output->set_release_solenoid(batter_output);
  output->set_voltage(winch_output);

  // WRITING TO QUEUES
  status_queue_->WriteMessage(status);
  output_queue_->WriteMessage(output);
}

}  // namespace climber
}  // namespace c2018
