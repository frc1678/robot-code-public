#include "c2018/subsystems/score_subsystem/wrist/wrist.h"
#include "gtest/gtest.h"

class WristTest : public ::testing::Test {
 public:
  WristTest() {
    plant_ = muan::control::StateSpacePlant<1, 3, 1>(
        frc1678::wrist_controller::controller::A(),
        frc1678::wrist_controller::controller::B(),
        frc1678::wrist_controller::controller::C());
  }
  // UPDATE
  void Update() {
    if (plant_.x(0) < 0) {
      plant_.x(0) = 0;
    }
    wrist_.Update(wrist_input_proto_, &wrist_output_proto_,
                  &wrist_status_proto_, outputs_enabled_);
    wrist_input_proto_->set_wrist_hall(plant_.x(0) >= 0.21 && plant_.x(0) <= 0.25);
    plant_.Update(
        (Eigen::Matrix<double, 1, 1>() << wrist_output_proto_->wrist_voltage())
            .finished());
  }

  void SetGoal(double angle, c2018::score_subsystem::IntakeMode intake_mode) {
    wrist_.SetGoal(angle, intake_mode);
  }

  c2018::score_subsystem::ScoreSubsystemInputProto wrist_input_proto_;
  c2018::score_subsystem::ScoreSubsystemOutputProto wrist_output_proto_;
  c2018::score_subsystem::ScoreSubsystemStatusProto wrist_status_proto_;
  c2018::score_subsystem::ScoreSubsystemGoalProto wrist_goal_proto_;

  double wrist_voltage_;
  bool outputs_enabled_;

 protected:
  muan::control::StateSpacePlant<1, 3, 1> plant_;

 private:
  c2018::score_subsystem::wrist::WristController wrist_;
};
// TESTS
TEST_F(WristTest, Calib) {
  wrist_input_proto_->set_wrist_encoder(0);
  wrist_input_proto_->set_wrist_hall(false);
  outputs_enabled_ = true;

  SetGoal(M_PI / 4, c2018::score_subsystem::IDLE);

  double offset = -M_PI / 2;

  for (int i = 0; i < 1000; i++) {
    wrist_input_proto_->set_wrist_encoder(plant_.x(0) + offset);
    Update();
    EXPECT_TRUE(wrist_output_proto_->wrist_voltage() < 12.01 || wrist_output_proto_->wrist_voltage() > -12.01);
  }

  EXPECT_NEAR(wrist_status_proto_->wrist_angle(), M_PI / 4, 0.01);
  EXPECT_TRUE(wrist_status_proto_->wrist_calibrated());
}

TEST_F(WristTest, EncoderFault) {
  wrist_input_proto_->set_wrist_encoder(0);
  wrist_input_proto_->set_wrist_hall(false);
  outputs_enabled_ = true;

  SetGoal(M_PI / 2, c2018::score_subsystem::IntakeMode::IDLE);

  for (int i = 0; i < 600; i++) {
    wrist_input_proto_->set_wrist_encoder(0);
    Update();

    EXPECT_TRUE(
        wrist_output_proto_->wrist_voltage() >=
        muan::utils::Cap(wrist_output_proto_->wrist_voltage(), -12, 12) - 0.01);
  }

  EXPECT_NEAR(wrist_output_proto_->wrist_voltage(), 0, 1e-2);
  EXPECT_NEAR(wrist_output_proto_->intake_voltage(), 0, 1e-2);
}

TEST_F(WristTest, IntakeEnabled) {
  wrist_input_proto_->set_wrist_encoder(0);
  wrist_input_proto_->set_wrist_hall(false);
  outputs_enabled_ = true;
  SetGoal(0.0, c2018::score_subsystem::IntakeMode::INTAKE);
  Update();

  EXPECT_NEAR(wrist_output_proto_->intake_voltage(), 12.0, 1e-3);
  EXPECT_FALSE(wrist_output_proto_->wrist_solenoid_open());
  EXPECT_FALSE(wrist_output_proto_->wrist_solenoid_close());
}
/*
TEST_F(WristTest, OuttakeEnabled) {
  outputs_enabled_ = true;
  CalibrationSequence();
  SetGoal(0.0, c2018::score_subsystem::IntakeMode::OUTTAKE);
  ReadMessages();
  RunFor(20);

  EXPECT_NEAR(wrist_output_proto_->intake_voltage(), -12.0, 1e-3);
  EXPECT_EQ(wrist_status_proto_->wrist_pinch(),
            c2018::score_subsystem::WRIST_IN);
  EXPECT_FALSE(wrist_output_proto_->wrist_solenoid_1());
  EXPECT_FALSE(wrist_output_proto_->wrist_solenoid_2());
}

TEST_F(WristTest, IdleEnabled) {
  outputs_enabled_ = true;
  CalibrationSequence();
  SetGoal(0.0, c2018::score_subsystem::IntakeMode::IDLE);
  ReadMessages();
  RunFor(20);

  EXPECT_EQ(wrist_status_proto_->wrist_pinch(),
            c2018::score_subsystem::WRIST_IN);
  EXPECT_FALSE(wrist_output_proto_->wrist_solenoid_1());
  EXPECT_FALSE(wrist_output_proto_->wrist_solenoid_2());
  EXPECT_EQ(wrist_output_proto_->intake_voltage(), 0);
}

TEST_F(WristTest, HoldEnabled) {
  outputs_enabled_ = true;
  CalibrationSequence();
  SetGoal(0.0, c2018::score_subsystem::IntakeMode::HOLD);
  ReadMessages();
  RunFor(20);

  EXPECT_EQ(wrist_status_proto_->wrist_pinch(),
            c2018::score_subsystem::WRIST_OUT);
  EXPECT_TRUE(wrist_output_proto_->wrist_solenoid_1());
  EXPECT_TRUE(wrist_output_proto_->wrist_solenoid_2());
  EXPECT_EQ(wrist_output_proto_->intake_voltage(), 0);
}

TEST_F(WristTest, Disabled) {
  outputs_enabled_ = false;
  EXPECT_EQ(wrist_output_proto_->wrist_voltage(), 0.0);
}

TEST_F(WristTest, Calibration) {
  outputs_enabled_ = true;
  wrist_input_proto_->set_wrist_hall(false);
  wrist_input_proto_->set_wrist_encoder(0);

  SetGoal(0.0, c2018::score_subsystem::IntakeMode::INTAKE);
  CalibrationSequence();

  EXPECT_TRUE(wrist_status_proto_->wrist_calibrated());
}

TEST_F(WristTest, SysIdle) {
  outputs_enabled_ = true;
  CalibrationSequence();
  ReadMessages();
  RunFor(20);

  EXPECT_EQ(wrist_output_proto_->wrist_voltage(), 0);
  EXPECT_EQ(wrist_output_proto_->intake_voltage(), 0);
  EXPECT_EQ(wrist_status_proto_->wrist_state(),
            c2018::score_subsystem::SYSTEM_IDLE);
}

TEST_F(WristTest, StateDisabled) {
  outputs_enabled_ = false;
  CalibrationSequence();
  ReadMessages();
  RunFor(20);

  EXPECT_EQ(wrist_output_proto_->intake_voltage(), 0);
  EXPECT_EQ(wrist_output_proto_->wrist_voltage(), 0);
  EXPECT_EQ(wrist_status_proto_->wrist_state(),
            c2018::score_subsystem::DISABLED);
}

TEST_F(WristTest, CanCapAngle) {
  outputs_enabled_ = true;
  CalibrationSequence();
  SetGoal(-5, c2018::score_subsystem::IntakeMode::IDLE);
  ReadMessages();
  RunFor(20);

  EXPECT_TRUE(wrist_status_proto_->wrist_position() >=
              muan::utils::Cap(wrist_status_proto_->wrist_position(), 0, M_PI) -
                  0.01);
}

TEST_F(WristTest, CanCapU) {
  outputs_enabled_ = true;
  wrist_voltage_ = -5;
  CalibrationSequence();
  ReadMessages();
  RunFor(20);

  EXPECT_TRUE(wrist_output_proto_->wrist_voltage() >=
              muan::utils::Cap(wrist_output_proto_->wrist_voltage(), -12, 12) -
                  0.01);
}

TEST_F(WristTest, CalibratingEnabled) {
  outputs_enabled_ = true;
  CalibrationSequence();
  ReadMessages();
  RunFor(20);

  EXPECT_EQ(wrist_output_proto_->wrist_voltage(), 0);
  EXPECT_EQ(wrist_output_proto_->intake_voltage(), 0);
  EXPECT_EQ(wrist_status_proto_->wrist_state(),
            c2018::score_subsystem::SYSTEM_IDLE);
}*/
