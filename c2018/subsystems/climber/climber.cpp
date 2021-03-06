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
  bool outputs_enabled = false;

  bool should_climb = false;

  bool batter_output = false;
  bool hook_output = false;
  double winch_output = 0.0;

  ClimberStatusProto status;
  ClimberOutputProto output;
  ClimberGoalProto goal;
  ClimberInputProto input;
  DriverStationProto ds;

  if (!goal_reader_.ReadLastMessage(&goal)) {
    goal->set_climber_goal(NONE);
  }
  if (!input_reader_.ReadLastMessage(&input)) {
    return;
  }

  ds_status_.ReadLastMessage(&ds);
  outputs_enabled = ds->is_sys_active();

  // SETTING STATUS AND SHOULD_CLIMB GOAL
  if (outputs_enabled) {
    switch (goal->climber_goal()) {
      case NONE:  // If not climbing
        should_climb = false;
        status->set_climber_state(IDLE);
        break;
      case APPROACHING:  // Hook up, drive forward to latch hook
        batter_output = false;
        hook_output = true;
        should_climb = false;
        status->set_climber_state(APPROACH);
        break;
      case BATTERING:  // Ramp down, arm down
        batter_output = true;
        hook_output = false;
        should_climb = false;
        status->set_climber_state(BATTER);
        break;
      case CLIMBING:  // Climb up with another robot
        batter_output = true;
        hook_output = false;
        should_climb = true;
        status->set_climber_state(CLIMB);
        break;
    }
  } else {
    status->set_climber_state(IDLE);
  }

  if (should_climb) {
    winch_output = 12.0;
  } else {
    winch_output = 0;
  }

  batter_output = batter_.Update(batter_output, outputs_enabled);

  // SETTING OUTPUTS
  output->set_batter_solenoid(batter_output);
  output->set_hook_solenoid(hook_output);
  output->set_voltage(winch_output);

  // WRITING TO QUEUES
  status_queue_->WriteMessage(status);
  output_queue_->WriteMessage(output);
}

}  // namespace climber
}  // namespace c2018
