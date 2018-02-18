#include "c2018/subsystems/score_subsystem/score_subsystem.h"
#include "gtest/gtest.h"
#include "muan/queues/queue_manager.h"

namespace c2018 {

namespace score_subsystem {

class ScoreSubsystemTest : public ::testing::Test {
 public:
  ScoreSubsystemTest() {
    wrist_plant_ = muan::control::StateSpacePlant<1, 3, 1>(
        frc1678::wrist_controller::controller::A(),
        frc1678::wrist_controller::controller::B(),
        frc1678::wrist_controller::controller::C());

    elevator_plant_ = muan::control::StateSpacePlant<1, 3, 1>(
        frc1678::elevator_controller::controller::first_stage_integral::A(),
        frc1678::elevator_controller::controller::first_stage_integral::B(),
        frc1678::elevator_controller::controller::first_stage_integral::C());
  }

  void UpdateInput() {
    double elevator = elevator_plant_.y(0);
    double wrist = wrist_plant_.y(0);
    score_subsystem_input_proto_->set_elevator_encoder(elevator +
                                                       elevator_offset_);
    score_subsystem_input_proto_->set_elevator_hall(
        std::abs(elevator - elevator::kHallEffectHeight) < 1e-2);
    score_subsystem_input_proto_->set_wrist_encoder(wrist + wrist_offset_);
    score_subsystem_input_proto_->set_wrist_hall(
        std::abs(wrist - wrist::kHallEffectAngle) < 1e-2);
  }

  void CalibrateDisabled() {
    driver_station_proto_->set_is_sys_active(false);

    for (int i = 0; i < 200; i++) {
      elevator_plant_.x(0) = i * 0.5e-2;
      elevator_plant_.x(1) = 1.0;
      wrist_plant_.x(0) = i * .125e-2;
      wrist_plant_.x(1) = 0.25;

      Update();
      if (i < 50) {
        EXPECT_EQ(score_subsystem_status_proto_->state(),
                  ScoreSubsystemState::CALIBRATING);
      }
    }

    EXPECT_TRUE(score_subsystem_status_proto_->elevator_calibrated());
    EXPECT_TRUE(score_subsystem_status_proto_->wrist_calibrated());
    EXPECT_EQ(score_subsystem_status_proto_->state(),
              ScoreSubsystemState::HOLDING);

    elevator_plant_.x(1) = 0.0;
    wrist_plant_.x(1) = 0.0;
  }

  void Update() {
    // Hard stops
    if (elevator_plant_.x(0) < 0) {
      elevator_plant_.x(0) = 0;
    }

    if (wrist_plant_.x(0) < 0) {
      wrist_plant_.x(0) = 0;
    }

    UpdateInput();

    WriteMessages();

    score_subsystem_.Update();

    SetWeights(elevator_plant_.x()(0, 0) >= 1.0,
               score_subsystem_input_proto_->has_cube());

    elevator_plant_.Update(
        (Eigen::Matrix<double, 1, 1>()
         << score_subsystem_output_proto_->elevator_voltage())
            .finished());
    wrist_plant_.Update((Eigen::Matrix<double, 1, 1>()
                         << score_subsystem_output_proto_->wrist_voltage())
                            .finished());

    ReadMessages();
  }

  void RunFor(int num_ticks) {
    for (int i = 0; i < num_ticks; i++) {
      Update();
    }
    LogicCheck();
  }

  void LogicCheck() {
    if (score_subsystem_status_proto_->elevator_actual_height() < 0.89 ||
        score_subsystem_status_proto_->elevator_unprofiled_goal() < 0.89) {
      EXPECT_LE(score_subsystem_status_proto_->wrist_unprofiled_goal(),
                 kWristSafeAngle);
      EXPECT_LE(wrist_plant_.x(0), kWristSafeAngle);
    }

    if (score_subsystem_status_proto_->wrist_angle() > M_PI / 2) {
      EXPECT_GE(score_subsystem_status_proto_->elevator_unprofiled_goal(),
                 kElevatorWristSafeHeight);
      EXPECT_GE(elevator_plant_.x(0), kElevatorWristSafeHeight);
    }

    EXPECT_NEAR(score_subsystem_output_proto_->elevator_voltage(), 0, 12);
    EXPECT_NEAR(score_subsystem_output_proto_->wrist_voltage(), 0, 12);
  }

  void ReadMessages() {
    score_subsystem_output_queue_.ReadLastMessage(
        &score_subsystem_output_proto_);
    score_subsystem_status_queue_.ReadLastMessage(
        &score_subsystem_status_proto_);
  }

  void WriteMessages() {
    score_subsystem_input_queue_->WriteMessage(score_subsystem_input_proto_);
    score_subsystem_goal_queue_->WriteMessage(score_subsystem_goal_proto_);
    driver_station_queue_->WriteMessage(driver_station_proto_);
  }

  void SetGoal(ScoreGoal score_goal, IntakeGoal intake_goal,
               bool outputs_enabled) {
    score_subsystem_goal_proto_->set_score_goal(score_goal);
    score_subsystem_goal_proto_->set_intake_goal(intake_goal);
    driver_station_proto_->set_is_sys_active(outputs_enabled);
  }

  void SetInput(double elevator_encoder, bool elevator_hall,
                double wrist_encoder, bool wrist_hall) {
    score_subsystem_input_proto_->set_elevator_encoder(elevator_encoder);
    score_subsystem_input_proto_->set_wrist_encoder(wrist_encoder);
    score_subsystem_input_proto_->set_elevator_hall(elevator_hall);
    score_subsystem_input_proto_->set_wrist_hall(wrist_hall);
  }

  bool outputs_enabled_;

  muan::wpilib::DriverStationQueue* driver_station_queue_ =
      muan::queues::QueueManager<muan::wpilib::DriverStationProto>::Fetch();

  ScoreSubsystemGoalQueue* score_subsystem_goal_queue_ =
      muan::queues::QueueManager<ScoreSubsystemGoalProto>::Fetch();

  ScoreSubsystemInputQueue* score_subsystem_input_queue_ =
      muan::queues::QueueManager<ScoreSubsystemInputProto>::Fetch();

  ScoreSubsystemStatusQueue::QueueReader score_subsystem_status_queue_ =
      muan::queues::QueueManager<ScoreSubsystemStatusProto>::Fetch()
          ->MakeReader();

  ScoreSubsystemOutputQueue::QueueReader score_subsystem_output_queue_ =
      muan::queues::QueueManager<ScoreSubsystemOutputProto>::Fetch()
          ->MakeReader();

  muan::wpilib::DriverStationProto driver_station_proto_;
  ScoreSubsystemGoalProto score_subsystem_goal_proto_;
  ScoreSubsystemInputProto score_subsystem_input_proto_;
  ScoreSubsystemStatusProto score_subsystem_status_proto_;
  ScoreSubsystemOutputProto score_subsystem_output_proto_;

 protected:
  muan::control::StateSpacePlant<1, 3, 1> elevator_plant_;
  muan::control::StateSpacePlant<1, 3, 1> wrist_plant_;

  double elevator_offset_ = 0.0;
  double wrist_offset_ = 0.0;

  void CheckGoal(double elevator, double wrist) const {
    EXPECT_NEAR(score_subsystem_status_proto_->elevator_actual_height(),
                elevator, 1e-3);
    EXPECT_NEAR(score_subsystem_status_proto_->wrist_angle(), wrist, 1e-3);
  }

 private:
  ScoreSubsystem score_subsystem_;

  void SetWeights(bool second_stage, bool has_cube) {
    if (second_stage && has_cube) {
      elevator_plant_.A() = frc1678::elevator_controller::controller::
          second_stage_cube_integral::A();
      elevator_plant_.B() = frc1678::elevator_controller::controller::
          second_stage_cube_integral::B();
      elevator_plant_.C() = frc1678::elevator_controller::controller::
          second_stage_cube_integral::C();
    } else if (second_stage && !has_cube) {
      elevator_plant_.A() =
          frc1678::elevator_controller::controller::second_stage_integral::A();
      elevator_plant_.B() =
          frc1678::elevator_controller::controller::second_stage_integral::B();
      elevator_plant_.C() =
          frc1678::elevator_controller::controller::second_stage_integral::C();
    } else if (!second_stage && has_cube) {
      elevator_plant_.A() = frc1678::elevator_controller::controller::
          first_stage_cube_integral::A();
      elevator_plant_.B() = frc1678::elevator_controller::controller::
          first_stage_cube_integral::B();
      elevator_plant_.C() = frc1678::elevator_controller::controller::
          first_stage_cube_integral::C();
    } else if (!second_stage && !has_cube) {
      elevator_plant_.A() =
          frc1678::elevator_controller::controller::first_stage_integral::A();
      elevator_plant_.B() =
          frc1678::elevator_controller::controller::first_stage_integral::B();
      elevator_plant_.C() =
          frc1678::elevator_controller::controller::first_stage_integral::C();
    }
  }
};

TEST_F(ScoreSubsystemTest, Disabled) {
  SetGoal(ScoreGoal::INTAKE_2, IntakeGoal::INTAKE, false);
  SetInput(0, false, 0, false);

  Update();

  EXPECT_EQ(score_subsystem_output_proto_->elevator_voltage(), 0);
  EXPECT_EQ(score_subsystem_output_proto_->intake_voltage(), 0);
  EXPECT_EQ(score_subsystem_output_proto_->wrist_voltage(), 0);
  EXPECT_FALSE(score_subsystem_output_proto_->wrist_solenoid_open());
  EXPECT_FALSE(score_subsystem_output_proto_->wrist_solenoid_close());
}

TEST_F(ScoreSubsystemTest, DisabledCalibrates) {
  elevator_offset_ = 1.0;
  wrist_offset_ = 1.0;
  CalibrateDisabled();
  RunFor(100);
  EXPECT_NEAR(score_subsystem_status_proto_->wrist_angle(), wrist_plant_.x(0),
              1e-2);
  EXPECT_NEAR(score_subsystem_status_proto_->elevator_actual_height(),
              elevator_plant_.x(0), 1e-2);
}

TEST_F(ScoreSubsystemTest, MoveTo) {
  CalibrateDisabled();

  // Intake 0
  SetGoal(ScoreGoal::INTAKE_0, IntakeGoal::INTAKE_NONE, true);
  RunFor(1);
  SetGoal(ScoreGoal::SCORE_NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(1000);
  CheckGoal(kElevatorIntake0, kWristForwardAngle);

  // Intake 1
  SetGoal(ScoreGoal::INTAKE_1, IntakeGoal::INTAKE_NONE, true);
  RunFor(1);
  SetGoal(ScoreGoal::SCORE_NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(1000);
  CheckGoal(kElevatorIntake1, kWristForwardAngle);

  // Intake 2
  SetGoal(ScoreGoal::INTAKE_2, IntakeGoal::INTAKE_NONE, true);
  RunFor(1);
  SetGoal(ScoreGoal::SCORE_NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(1000);
  CheckGoal(kElevatorIntake2, kWristForwardAngle);

  // Force stow
  SetGoal(ScoreGoal::STOW, IntakeGoal::INTAKE_NONE, true);
  RunFor(1);
  SetGoal(ScoreGoal::SCORE_NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(1000);
  CheckGoal(kElevatorStow, kWristStowAngle);

  // Switch
  SetGoal(ScoreGoal::SWITCH, IntakeGoal::INTAKE_NONE, true);
  RunFor(1);
  SetGoal(ScoreGoal::SCORE_NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(1000);
  CheckGoal(kElevatorSwitch, kWristForwardAngle);

  // Scale low forward
  SetGoal(ScoreGoal::SCALE_LOW_FORWARD, IntakeGoal::INTAKE_NONE, true);
  RunFor(1);
  SetGoal(ScoreGoal::SCORE_NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(1000);
  CheckGoal(kElevatorScaleLow, kWristForwardAngle);

  // Scale low reverse
  SetGoal(ScoreGoal::SCALE_LOW_REVERSE, IntakeGoal::INTAKE_NONE, true);
  RunFor(1);
  SetGoal(ScoreGoal::SCORE_NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(1000);
  CheckGoal(kElevatorScaleLow + kElevatorReversedOffset, kWristBackwardAngle);

  // Scale mid forward
  SetGoal(ScoreGoal::SCALE_MID_FORWARD, IntakeGoal::INTAKE_NONE, true);
  RunFor(1);
  SetGoal(ScoreGoal::SCORE_NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(1000);
  CheckGoal(kElevatorScaleMid, kWristForwardAngle);

  // Scale mid reverse
  SetGoal(ScoreGoal::SCALE_MID_REVERSE, IntakeGoal::INTAKE_NONE, true);
  RunFor(1);
  SetGoal(ScoreGoal::SCORE_NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(1000);
  CheckGoal(kElevatorScaleMid + kElevatorReversedOffset, kWristBackwardAngle);

  // Scale high forward
  SetGoal(ScoreGoal::SCALE_HIGH_FORWARD, IntakeGoal::INTAKE_NONE, true);
  RunFor(1);
  SetGoal(ScoreGoal::SCORE_NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(1000);
  CheckGoal(kElevatorScaleHigh, kWristForwardAngle);

  // Scale high reverse
  SetGoal(ScoreGoal::SCALE_HIGH_REVERSE, IntakeGoal::INTAKE_NONE, true);
  RunFor(1);
  SetGoal(ScoreGoal::SCORE_NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(1000);
  CheckGoal(kElevatorScaleHigh + kElevatorReversedOffset, kWristBackwardAngle);
}

TEST_F(ScoreSubsystemTest, IntakeManual) {
  CalibrateDisabled();

  SetGoal(ScoreGoal::INTAKE_0, IntakeGoal::INTAKE, true);
  Update();

  EXPECT_EQ(score_subsystem_output_proto_->intake_voltage(),
            wrist::kIntakeVoltage);
  EXPECT_EQ(score_subsystem_status_proto_->state(), INTAKING);

  SetGoal(ScoreGoal::INTAKE_0, IntakeGoal::FORCE_STOP, true);
  Update();

  EXPECT_EQ(score_subsystem_output_proto_->intake_voltage(), 0);
  EXPECT_EQ(score_subsystem_status_proto_->state(), HOLDING);
}

TEST_F(ScoreSubsystemTest, OuttakeManual) {
  CalibrateDisabled();

  SetGoal(ScoreGoal::INTAKE_0, IntakeGoal::OUTTAKE, true);
  Update();

  EXPECT_EQ(score_subsystem_output_proto_->intake_voltage(),
            wrist::kOuttakeVoltage);
  EXPECT_EQ(score_subsystem_status_proto_->state(), SCORING);

  SetGoal(ScoreGoal::INTAKE_0, IntakeGoal::FORCE_STOP, true);
  Update();

  EXPECT_EQ(score_subsystem_output_proto_->intake_voltage(), 0);
  EXPECT_EQ(score_subsystem_status_proto_->state(), HOLDING);
}

TEST_F(ScoreSubsystemTest, IntakeToHolding) {
  CalibrateDisabled();

  SetGoal(ScoreGoal::INTAKE_0, IntakeGoal::INTAKE, true);
  Update();

  SetGoal(ScoreGoal::SCORE_NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(500);

  EXPECT_EQ(score_subsystem_output_proto_->intake_voltage(),
            wrist::kIntakeVoltage);
  EXPECT_EQ(score_subsystem_status_proto_->state(), INTAKING);

  score_subsystem_input_proto_->set_has_cube(true);
  RunFor(600);

  EXPECT_EQ(score_subsystem_output_proto_->intake_voltage(),
            wrist::kHoldingVoltage);
}

TEST_F(ScoreSubsystemTest, ScoreToIdle) {
  score_subsystem_input_proto_->set_has_cube(true);

  CalibrateDisabled();

  SetGoal(ScoreGoal::SCALE_MID_REVERSE, IntakeGoal::INTAKE_NONE, true);
  Update();
  SetGoal(ScoreGoal::SCORE_NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(600);
  CheckGoal(kElevatorScaleMid + kElevatorReversedOffset, kWristBackwardAngle);

  SetGoal(ScoreGoal::SCORE_NONE, IntakeGoal::OUTTAKE, true);
  Update();
  SetGoal(ScoreGoal::SCORE_NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(10);

  EXPECT_EQ(score_subsystem_output_proto_->intake_voltage(),
            wrist::kOuttakeVoltage);
  EXPECT_EQ(score_subsystem_status_proto_->state(), SCORING);

  score_subsystem_input_proto_->set_has_cube(false);
  RunFor(500);

  EXPECT_EQ(score_subsystem_output_proto_->intake_voltage(), 0);
}

TEST_F(ScoreSubsystemTest, ForceIntake) {
  CalibrateDisabled();

  score_subsystem_input_proto_->set_has_cube(true);
  SetGoal(ScoreGoal::INTAKE_0, IntakeGoal::INTAKE, true);
  RunFor(500);

  EXPECT_EQ(score_subsystem_output_proto_->intake_voltage(),
            wrist::kIntakeVoltage);
  EXPECT_EQ(score_subsystem_status_proto_->state(), INTAKING);
  CheckGoal(kElevatorIntake0, kWristForwardAngle);
}

}  // namespace score_subsystem
}  // namespace c2018
