#include "c2019/subsystems/superstructure/superstructure.h"

#include <algorithm>

namespace c2019 {
namespace superstructure {

using muan::queues::QueueManager;
using muan::wpilib::DriverStationProto;

Superstructure::Superstructure()
    : goal_reader_{QueueManager<SuperstructureGoalProto>::Fetch()
                       ->MakeReader()},
      input_reader_{
          QueueManager<SuperstructureInputProto>::Fetch()->MakeReader()},
      status_queue_{QueueManager<SuperstructureStatusProto>::Fetch()},
      output_queue_{QueueManager<SuperstructureOutputProto>::Fetch()},
      ds_status_reader_{
          QueueManager<DriverStationProto>::Fetch()->MakeReader()} {}

void Superstructure::BoundGoal(double* elevator_goal, double* wrist_goal) {
  // If wrist angle is higher than safe angle, cap elevator to safe height
  if (wrist_status_->wrist_angle() > kWristSafeAngle) {
    *elevator_goal = muan::utils::Cap(*elevator_goal, 0, kElevatorSafeHeight);
  }

  // If elevator is higher than safe height, cap wrist to safe angle
  if (elevator_status_->elevator_height() > kElevatorSafeHeight) {
    *wrist_goal =
        muan::utils::Cap(*wrist_goal, kWristMinAngle, kWristSafeAngle);
  }
}

elevator::ElevatorGoalProto Superstructure::PopulateElevatorGoal() {
  elevator::ElevatorGoalProto goal;
  goal->set_height(elevator_height_);
  goal->set_crawling(crawling_);
  goal->set_high_gear(high_gear_);
  goal->set_crawler_down(crawler_down_);
  goal->set_brake(brake_);
  return goal;
}

wrist::WristGoalProto Superstructure::PopulateWristGoal() {
  wrist::WristGoalProto goal;
  goal->set_angle(wrist_angle_);
  return goal;
}

ground_hatch_intake::GroundHatchIntakeGoalProto
Superstructure::PopulateGroundHatchIntakeGoal() {
  ground_hatch_intake::GroundHatchIntakeGoalProto goal;
  if (intake_goal_ == INTAKE_GROUND_HATCH) {
    goal->set_goal(ground_hatch_intake::REQUEST_HATCH);
  } else if (intake_goal_ == OUTTAKE_GROUND_HATCH || intake_goal_ == SPIT) {
    goal->set_goal(ground_hatch_intake::EJECT);
  } else if (intake_goal_ == POP) {
    goal->set_goal(ground_hatch_intake::RISE);
  } else {
    goal->set_goal(ground_hatch_intake::NONE);
  }
  return goal;
}

hatch_intake::HatchIntakeGoalProto Superstructure::PopulateHatchIntakeGoal() {
  hatch_intake::HatchIntakeGoalProto goal;
  if (intake_goal_ == INTAKE_HATCH || intake_goal_ == POP ||
      intake_goal_ == SPIT) {
    goal->set_goal(hatch_intake::INTAKE);
  } else if (intake_goal_ == OUTTAKE_HATCH) {
    goal->set_goal(hatch_intake::SCORE);
  } else {
    goal->set_goal(hatch_intake::NONE);
  }
  return goal;
}

cargo_intake::CargoIntakeGoalProto Superstructure::PopulateCargoIntakeGoal() {
  cargo_intake::CargoIntakeGoalProto goal;
  if (intake_goal_ == INTAKE_CARGO) {
    goal->set_goal(cargo_intake::INTAKE);
  } else if (intake_goal_ == OUTTAKE_CARGO) {
    goal->set_goal(cargo_intake::OUTTAKE);
  } else {
    goal->set_goal(cargo_intake::IDLE);
  }
  return goal;
}

winch::WinchGoalProto Superstructure::PopulateWinchGoal() {
  winch::WinchGoalProto goal;
  goal->set_winch(should_climb_);
  // Buddy climb logic
  if (should_climb_) {
    if (buddy_) {
      goal->set_climb_goal(winch::BUDDY);
    } else {
      goal->set_climb_goal(winch::SOLO);
    }
  } else {
    goal->set_climb_goal(winch::NONE);
  }
  return goal;
}

void Superstructure::Update() {
  cargo_intake::CargoIntakeInputProto cargo_intake_input;
  elevator::ElevatorInputProto elevator_input;
  ground_hatch_intake::GroundHatchIntakeInputProto ground_hatch_intake_input;
  hatch_intake::HatchIntakeInputProto hatch_intake_input;
  wrist::WristInputProto wrist_input;
  winch::WinchInputProto winch_input;

  cargo_intake::CargoIntakeOutputProto cargo_intake_output;
  elevator::ElevatorOutputProto elevator_output;
  ground_hatch_intake::GroundHatchIntakeOutputProto ground_hatch_intake_output;
  hatch_intake::HatchIntakeOutputProto hatch_intake_output;
  wrist::WristOutputProto wrist_output;
  winch::WinchOutputProto winch_output;

  SuperstructureGoalProto goal;
  SuperstructureInputProto input;
  SuperstructureOutputProto output;
  DriverStationProto driver_station;

  if (!input_reader_.ReadLastMessage(&input)) {
    // TODO(Kyle) handle this gracefully
    return;
  }

  if (!ds_status_reader_.ReadLastMessage(&driver_station)) {
    // Even if we don't get a message, we know that it is a 12V battery
    driver_station->set_battery_voltage(12.0);
  }

  while (goal_reader_.ReadMessage(&goal)) {
    // Bridge between score goal enumerator and the individual mechanism goals
    SetGoal(goal);
    // All the logic in the state machine is in this function
    RunStateMachine();
  }

  double capped_elevator_height = elevator_height_;
  double capped_wrist_angle = wrist_angle_;
  // Now we make them safe so stuff doesn't break
  BoundGoal(&capped_elevator_height, &capped_wrist_angle);

  hatch_intake_input->set_hatch_proxy(input->hatch_intake_proxy());
  cargo_intake_input->set_has_cargo(input->cargo_proxy());
  ground_hatch_intake_input->set_current(input->hatch_ground_current());
  elevator_input->set_elevator_encoder(input->elevator_encoder());
  elevator_input->set_zeroed(input->elevator_zeroed());
  wrist_input->set_wrist_encoder(input->wrist_encoder());
  wrist_input->set_wrist_hall(input->wrist_hall());

  auto elevator_goal = PopulateElevatorGoal();
  elevator_goal->set_height(capped_elevator_height);
  auto wrist_goal = PopulateWristGoal();
  wrist_goal->set_angle(capped_wrist_angle);
  auto ground_hatch_intake_goal = PopulateGroundHatchIntakeGoal();
  auto hatch_intake_goal = PopulateHatchIntakeGoal();
  auto cargo_intake_goal = PopulateCargoIntakeGoal();
  auto winch_goal = PopulateWinchGoal();

  // Then we tell the controller to do it
  elevator_.SetGoal(elevator_goal);
  elevator_.Update(elevator_input, &elevator_output, &elevator_status_,
                   driver_station->is_sys_active());

  wrist_.SetGoal(wrist_goal);
  wrist_.Update(wrist_input, &wrist_output, &wrist_status_,
                driver_station->is_sys_active());

  ground_hatch_intake_.SetGoal(ground_hatch_intake_goal);
  ground_hatch_intake_.Update(
      ground_hatch_intake_input, &ground_hatch_intake_output,
      &ground_hatch_intake_status_, driver_station->is_sys_active());

  hatch_intake_.SetGoal(hatch_intake_goal);
  hatch_intake_.Update(hatch_intake_input, &hatch_intake_output,
                       &hatch_intake_status_, driver_station->is_sys_active());

  cargo_intake_.SetGoal(cargo_intake_goal);
  cargo_intake_.Update(cargo_intake_input, &cargo_intake_output,
                       &cargo_intake_status_, driver_station->is_sys_active());

  winch_.SetGoal(winch_goal);
  winch_.Update(winch_input, &winch_output, &winch_status_,
                driver_station->is_sys_active());

  status_->set_state(state_);
  status_->set_has_ground_hatch(ground_hatch_intake_status_->has_hatch());
  status_->set_ground_hatch_intake_state(static_cast<GroundHatchIntakeState>(
      ground_hatch_intake_status_->state()));
  status_->set_hatch_intake_state(
      static_cast<HatchIntakeState>(hatch_intake_status_->state()));
  status_->set_has_hp_hatch(hatch_intake_status_->has_hatch());
  status_->set_cargo_intake_state(
      static_cast<CargoIntakeState>(cargo_intake_status_->state()));
  status_->set_has_cargo(cargo_intake_status_->has_cargo());
  status_->set_climb_type(static_cast<ClimbType>(winch_status_->climb_type()));
  status_->set_climb(winch_status_->climb());
  status_->set_winch_current(winch_status_->winch_current());
  status_->set_elevator_is_calibrated(elevator_status_->is_calibrated());
  status_->set_wrist_is_calibrated(wrist_status_->is_calibrated());
  status_->set_elevator_goal(elevator_height_);
  status_->set_wrist_goal(wrist_angle_);
  status_->set_wrist_angle(wrist_status_->wrist_angle());
  status_->set_elevator_height(elevator_status_->elevator_height());

  output->set_arrow_solenoid(hatch_intake_output->flute_solenoid());
  output->set_backplate_solenoid(hatch_intake_output->backplate_solenoid());
  output->set_cargo_roller_voltage(cargo_intake_output->roller_voltage());
  output->set_hatch_roller_voltage(
      ground_hatch_intake_output->roller_voltage());
  output->set_snap_down(ground_hatch_intake_output->snap_down());
  output->set_winch_voltage(winch_output->winch_voltage());
  output->set_drop_forks(winch_output->drop_forks());
  output->set_elevator_high_gear(elevator_output->high_gear());
  output->set_crawler_solenoid(elevator_output->crawler_solenoid());
  output->set_crawler_voltage(elevator_output->crawler_voltage());
  output->set_brake(elevator_output->brake());
  output->set_elevator_setpoint(elevator_output->elevator_setpoint());
  output->set_elevator_setpoint_type(
      static_cast<TalonOutput>(elevator_output->elevator_output_type()));
  output->set_wrist_setpoint(wrist_output->wrist_setpoint());
  output->set_wrist_setpoint_type(
      static_cast<TalonOutput>(wrist_output->output_type()));

  // Write those queues after Updating the controllers
  status_queue_->WriteMessage(status_);
  output_queue_->WriteMessage(output);
}

void Superstructure::SetGoal(const SuperstructureGoalProto& goal) {
  // These set the member variable goals before they are constrained
  // They are set based on the score goal enumerator
  switch (goal->score_goal()) {
    case NONE:
      break;
    case CARGO_ROCKET_FIRST:
      elevator_height_ = kCargoRocketFirstHeight;
      wrist_angle_ = kCargoRocketFirstAngle;
      break;
    case CARGO_ROCKET_BACKWARDS:
      elevator_height_ = kCargoRocketBackwardsHeight;
      wrist_angle_ = kCargoRocketBackwardsAngle;
      break;
    case CARGO_ROCKET_SECOND:
      elevator_height_ = kCargoRocketSecondHeight;
      wrist_angle_ = kCargoRocketSecondAngle;
      break;
    case CARGO_ROCKET_THIRD:
      elevator_height_ = kCargoRocketThirdHeight;
      wrist_angle_ = kCargoRocketThirdAngle;
      break;
    case CARGO_SHIP_FORWARDS:
      elevator_height_ = kCargoShipForwardsHeight;
      wrist_angle_ = kCargoShipForwardsAngle;
      break;
    case CARGO_SHIP_BACKWARDS:
      elevator_height_ = kCargoShipBackwardsHeight;
      wrist_angle_ = kCargoShipBackwardsAngle;
      break;
    case HATCH_ROCKET_FIRST:
      elevator_height_ = kHatchRocketFirstHeight;
      wrist_angle_ = kHatchForwardsAngle;
      break;
    case HATCH_ROCKET_BACKWARDS:
      elevator_height_ = kHatchRocketBackwardsHeight;
      wrist_angle_ = kHatchBackwardsAngle;
      break;
    case HATCH_ROCKET_SECOND:
      elevator_height_ = kHatchRocketSecondHeight;
      wrist_angle_ = kHatchForwardsAngle;
      break;
    case HATCH_ROCKET_THIRD:
      elevator_height_ = kHatchRocketThirdHeight;
      wrist_angle_ = kHatchForwardsAngle;
      break;
    case HATCH_SHIP_FORWARDS:
      elevator_height_ = kHatchShipForwardsHeight;
      wrist_angle_ = kHatchForwardsAngle;
      break;
    case HATCH_SHIP_BACKWARDS:
      elevator_height_ = kHatchShipBackwardsHeight;
      wrist_angle_ = kHatchBackwardsAngle;
      break;
    case HANDOFF:
      elevator_height_ = kHandoffHeight;
      wrist_angle_ = kHandoffAngle;
      break;
    case STOW:
      elevator_height_ = kStowHeight;
      wrist_angle_ = kStowAngle;
      break;
    case CARGO_GROUND:
      elevator_height_ = kCargoGroundHeight;
      wrist_angle_ = kCargoGroundAngle;
    case CLIMB:
      GoToState(CLIMBING);
      elevator_height_ = kClimbHeight;
      wrist_angle_ = kClimbAngle;
      should_climb_ = true;
      buddy_ = false;
      high_gear_ = false;
      crawler_down_ = true;
      break;
    case BUDDY_CLIMB:
      GoToState(BUDDY_CLIMBING);
      elevator_height_ = kClimbHeight;
      wrist_angle_ = kClimbAngle;
      should_climb_ = true;
      buddy_ = true;
      high_gear_ = false;
      break;
    case CRAWL:
      crawling_ = true;
      high_gear_ = false;
      crawler_down_ = true;
      break;
    case CRAWL_BRAKED:
      crawling_ = true;
      high_gear_ = false;
      crawler_down_ = true;
      brake_ = true;
      break;
    case BRAKE:
      brake_ = true;
      break;
  }

  // Godmode
  elevator_height_ += goal->elevator_god_mode_goal() * 0.005;
  wrist_angle_ += goal->wrist_god_mode_goal() * 0.005;

  elevator_height_ = muan::utils::Cap(elevator_height_, kElevatorMinHeight,
                                      kElevatorMaxHeight);
  wrist_angle_ = muan::utils::Cap(wrist_angle_, kWristMinAngle, kWristMaxAngle);

  switch (goal->intake_goal()) {
    case INTAKE_NONE:
      intake_goal_ = goal->intake_goal();
      break;
    case INTAKE_IDLE:
      GoToState(IDLE, goal->intake_goal());
      break;
    case INTAKE_HATCH:
      GoToState(INTAKING_HATCH, goal->intake_goal());
      if (hatch_intake_status_->has_hatch()) {
        GoToState(IDLE, goal->intake_goal());
      }
      break;
    case INTAKE_GROUND_HATCH:
      GoToState(INTAKING_GROUND_HATCH, goal->intake_goal());
      if (ground_hatch_intake_status_->has_hatch()) {
        GoToState(IDLE, goal->intake_goal());
      }
      break;
    case INTAKE_CARGO:
      GoToState(INTAKING_CARGO, goal->intake_goal());
      if (cargo_intake_status_->has_cargo()) {
        GoToState(IDLE, goal->intake_goal());
      }
      break;
    case OUTTAKE_HATCH:
      if (!hatch_intake_status_->has_hatch()) {
        GoToState(IDLE, goal->intake_goal());
      }
      break;
    case OUTTAKE_GROUND_HATCH:
      GoToState(IDLE, goal->intake_goal());
      break;
    case OUTTAKE_CARGO:
      if (!cargo_intake_status_->has_cargo()) {
        GoToState(IDLE, goal->intake_goal());
      }
      break;
    case POP:
      if (ground_hatch_intake_status_->has_hatch() &&
          std::abs(elevator_status_->elevator_height() - kHandoffHeight) <
              kElevatorHandoffTolerance &&
          std::abs(wrist_status_->wrist_angle() - kHandoffAngle) <
              kWristHandoffTolerance) {
        GoToState(HANDING_OFF, goal->intake_goal());
        if (hatch_intake_status_->has_hatch()) {
          intake_goal_ = SPIT;
        }
      } else {
        GoToState(IDLE);
      }
      break;
    case SPIT:
      elevator_height_ = kSpitHeight;
      GoToState(HANDING_OFF, goal->intake_goal());
      if (hatch_intake_status_->has_hatch()) {
        GoToState(IDLE);
      }
      break;
  }
}

void Superstructure::RunStateMachine() {
  switch (state_) {
    case CALIBRATING:
      elevator_height_ = elevator_status_->elevator_height();
      wrist_angle_ = wrist_status_->wrist_angle();
      if (elevator_status_->is_calibrated() && wrist_status_->is_calibrated()) {
        GoToState(IDLE);
      }
      break;
    case INTAKING_GROUND_HATCH:
      if (ground_hatch_intake_status_->has_hatch()) {
        GoToState(IDLE);
      }
      break;
    case INTAKING_HATCH:
      if (hatch_intake_status_->has_hatch()) {
        GoToState(IDLE);
      }
      break;
    case INTAKING_CARGO:
      if (cargo_intake_status_->has_cargo()) {
        GoToState(IDLE);
      }
      break;
    case IDLE:
      break;
    case HANDING_OFF:
      if (hatch_intake_status_->has_hatch()) {
        GoToState(IDLE);
      }
      break;
    case CLIMBING:
      break;
    case BUDDY_CLIMBING:
      break;
  }
}

void Superstructure::GoToState(SuperstructureState desired_state,
                               IntakeGoal intake) {
  switch (state_) {
    case CALIBRATING:
      if (wrist_status_->is_calibrated() && elevator_status_->is_calibrated()) {
        state_ = desired_state;
      } else {
        LOG(ERROR, "Tried to go to invalid state %d while calibrating!",
            static_cast<int>(desired_state));
      }
      break;
    case INTAKING_GROUND_HATCH:
      state_ = desired_state;
      break;
    case INTAKING_HATCH:
      state_ = desired_state;
      break;
    case INTAKING_CARGO:
      state_ = desired_state;
      break;
    case IDLE:
      if (desired_state == INTAKING_GROUND_HATCH ||
          desired_state == INTAKING_HATCH || desired_state == INTAKING_CARGO) {
        if (intake == IntakeGoal::INTAKE_GROUND_HATCH ||
            intake == IntakeGoal::INTAKE_HATCH ||
            intake == IntakeGoal::INTAKE_CARGO) {
          intake_goal_ = intake;
        } else {
          LOG(ERROR,
              "Tried to go to invalid state/intake_goal combination %d, %d",
              static_cast<int>(desired_state), static_cast<int>(intake));
        }
      } else {
        if (intake == IntakeGoal::INTAKE_NONE ||
            intake == IntakeGoal::INTAKE_IDLE ||
            intake == IntakeGoal::OUTTAKE_HATCH ||
            intake == IntakeGoal::OUTTAKE_GROUND_HATCH ||
            intake == IntakeGoal::OUTTAKE_CARGO || intake == IntakeGoal::POP ||
            intake == IntakeGoal::SPIT) {
          intake_goal_ = intake;
        } else {
          LOG(ERROR,
              "Tried to go to invalid state/intake_goal combination %d, %d",
              static_cast<int>(desired_state), static_cast<int>(intake));
        }
      }
      state_ = desired_state;
      break;
    case HANDING_OFF:
      if (desired_state == HANDING_OFF) {
        if (intake == IntakeGoal::POP || intake == IntakeGoal::SPIT) {
          intake_goal_ = intake;
        } else {
          LOG(ERROR,
              "Tried to go to invalid state/intake_goal combination %d, %d",
              static_cast<int>(desired_state), static_cast<int>(intake));
        }
        state_ = desired_state;
        break;
      }
    case CLIMBING:
      state_ = desired_state;
      break;
    case BUDDY_CLIMBING:
      state_ = desired_state;
      break;
  }
}

}  // namespace superstructure
}  // namespace c2019
