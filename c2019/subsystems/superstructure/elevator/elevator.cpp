#include "c2019/subsystems/superstructure/elevator/elevator.h"

namespace c2019 {
namespace elevator {

Elevator::Elevator() {}

void Elevator::SetGoal(const ElevatorGoalProto& goal) {
  height_goal_ =
      muan::utils::Cap(goal->height(), kMinHeight, kMaxHeight);
  high_gear_ = goal->high_gear();
  wants_brake_ = goal->brake();
  crawling_ = goal->crawling();
  crawler_down_ = goal->crawler_down();
}

void Elevator::Update(bool outputs_enabled, const ElevatorInputProto& input,
                      ElevatorOutputProto* output,
                      ElevatorStatusProto* status) {
  auto braked = disk_brake_.Update(wants_brake_ && outputs_enabled);

  if (prev_encoder_ == input->elevator_encoder() &&
      input->elevator_voltage() > kMinFaultVoltage) {
    num_encoder_fault_ticks_++;
    if (num_encoder_fault_ticks_ >= kMinFaultTicks) {
      encoder_fault_ = true;
    }
  } else {
    num_encoder_fault_ticks_ = 0;
  }

  prev_encoder_ = input->elevator_encoder();

  if (input->zeroed() && outputs_enabled) {
    if (braked != muan::DiskBrake::LOCKED) {
      (*output)->set_elevator_output_type(POSITION);
      (*output)->set_elevator_setpoint(height_goal_);
      (*output)->set_elevator_ff(CalculateFeedForwards(
          input->has_cargo(), input->has_hatch(),
          input->elevator_encoder() > kSecondStageHeight));
    } else {
      (*output)->set_elevator_output_type(OPEN_LOOP);
      (*output)->set_elevator_setpoint(0.);
      (*output)->set_elevator_ff(0.);
    }
    (*output)->set_brake(wants_brake_);
    (*output)->set_high_gear(high_gear_);

    (*output)->set_crawler_solenoid(crawler_down_);
    (*output)->set_crawler_voltage(crawling_ ? 12. : 0.);
  } else {
    (*output)->set_elevator_output_type(OPEN_LOOP);
    (*output)->set_elevator_setpoint(0.);
    (*output)->set_elevator_ff(0.);
    (*output)->set_high_gear(false);

    (*output)->set_crawler_solenoid(false);
    (*output)->set_crawler_voltage(0);
    (*output)->set_brake(false);
  }

  (*status)->set_braked(braked == muan::DiskBrake::LOCKED);
  (*status)->set_height(input->elevator_encoder());
  (*status)->set_is_calibrated(input->zeroed());
  (*status)->set_elevator_velocity(input->elevator_velocity());
  (*status)->set_elevator_at_top(input->elevator_encoder() == kMaxHeight);
  (*status)->set_elevator_goal(height_goal_);
  (*status)->set_elevator_encoder_fault_detected(false);
}

double Elevator::CalculateFeedForwards(bool has_cargo, bool has_panel,
                                       bool second_stage) {
  double ff = kFF;
  if (has_cargo) {
    ff += kFFCargo;
  }
  if (has_panel) {
    ff += kFFHatch;
  }
  if (second_stage) {
    ff += kFFSecondStage;
  }

  return ff;
}

}  // namespace elevator
}  // namespace c2019
