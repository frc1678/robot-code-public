#include "wrist.h"
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

    wrist_.Update(wrist_input_proto_, &wrist_output_proto_, &wrist_status_proto_, outputs_enabled_);
    plant_.Update(
        (Eigen::Matrix<double, 1, 1>() << wrist_output_proto_->wrist_voltage())
            .finished());
    wrist_input_proto_->set_wrist_hall(plant_.x(0) >= 0.04 && plant_.x(0) <= 0.06);
  }

  void SetGoal() {
    wrist_.SetGoal(angle_, intake_mode_);
  }

  void SetInput(double position, bool hall, double current, bool has_cube) {
    wrist_input_proto_->set_wrist_encoder(position);
    wrist_input_proto_->set_wrist_hall(hall);
    wrist_input_proto_->set_intake_current(current);
    wrist_input_proto_->set_has_cube(has_cube);
    wrist_input_proto_->set_wrist_encoder(plant_.x(0));
  }

  void CalibrationSequence() {
    double offset = 0.5; //TODO (Mohamed) make it the real number when it is made
    for (int i = 0; i < 2000; i++) {
      wrist_input_proto_->set_wrist_encoder(plant_.x(0) + offset);
      Update();
      EXPECT_TRUE(wrist_output_proto_->wrist_voltage() >=
                  muan::utils::Cap(wrist_output_proto_->wrist_voltage(), -12, 12) - 0.01);
    }
  }

  c2018::score_subsystem::ScoreSubsystemInputProto wrist_input_proto_;
  c2018::score_subsystem::ScoreSubsystemOutputProto wrist_output_proto_;
  c2018::score_subsystem::ScoreSubsystemStatusProto wrist_status_proto_;
  c2018::score_subsystem::ScoreSubsystemGoalProto wrist_goal_proto_;
  c2018::score_subsystem::IntakeMode intake_mode_;

  double angle_;
  double wrist_voltage_;
  bool outputs_enabled_;
protected:
  muan::control::StateSpacePlant<1, 3, 1> plant_;
 private:
  c2018::score_subsystem::wrist::WristController wrist_;
};
// TESTS
TEST_F(WristTest, IsSane) {
  plant_.Update((Eigen::Matrix<double, 1, 1>() << 0.).finished());
  EXPECT_NEAR(wrist_output_proto_->wrist_voltage(), 0., 12.);

  EXPECT_NEAR(plant_.x(0), 0, 1e-5);
  EXPECT_NEAR(plant_.x(1), 0, 1e-5);
  EXPECT_NEAR(plant_.x(2), 0, 1e-5);
}

TEST_F(WristTest, Fault) {
  outputs_enabled_ = true;
  angle_ = M_PI / 2;
  intake_mode_ = c2018::score_subsystem::IntakeMode::IDLE;
  SetGoal();

  for (int i = 0; i < 600; i++) {
    wrist_input_proto_->set_wrist_encoder(0);
    Update();
    EXPECT_TRUE(wrist_output_proto_->wrist_voltage() >=
                muan::utils::Cap(wrist_output_proto_->wrist_voltage(), -12, 12) - 0.01);
  }

  EXPECT_NEAR(wrist_output_proto_->wrist_voltage(), 0, 1e-2);
  EXPECT_NEAR(wrist_output_proto_->roller_voltage(), 0, 1e-2);
  EXPECT_EQ(wrist_status_proto_->wrist_state(), c2018::score_subsystem::ENCODER_FAULT);
}


TEST_F(WristTest, IntakeEnabled) {
  outputs_enabled_ = true;
  CalibrationSequence();
  intake_mode_ = c2018::score_subsystem::IntakeMode::INTAKE;
  SetGoal();
  Update();

  EXPECT_NEAR(wrist_output_proto_->roller_voltage(), 12.0, 1e-3);
  EXPECT_EQ(wrist_output_proto_->wrist_pinch(), c2018::score_subsystem::WRIST_OUT);
}

TEST_F(WristTest, OuttakeEnabled) {
  outputs_enabled_ = true;
  CalibrationSequence();
  intake_mode_ = c2018::score_subsystem::IntakeMode::OUTTAKE;
  SetGoal();
  Update();

  EXPECT_NEAR(wrist_output_proto_->roller_voltage(), -12.0, 1e-3);
  EXPECT_EQ(wrist_output_proto_->wrist_pinch(), c2018::score_subsystem::WRIST_IN);
}

TEST_F(WristTest, IdleEnabled) {
  outputs_enabled_ = true;
  CalibrationSequence();
  intake_mode_ = c2018::score_subsystem::IntakeMode::IDLE;
  SetGoal();
  Update();

  EXPECT_EQ(wrist_output_proto_->wrist_pinch(), c2018::score_subsystem::WRIST_IN);
  EXPECT_EQ(wrist_output_proto_->roller_voltage(), 0);
}

TEST_F(WristTest, HoldEnabled) {
  outputs_enabled_ = true;
  CalibrationSequence();
  intake_mode_ = c2018::score_subsystem::IntakeMode::HOLD;
  SetGoal();
  Update();

  EXPECT_EQ(wrist_output_proto_->wrist_pinch(), c2018::score_subsystem::WRIST_OUT);
  EXPECT_EQ(wrist_output_proto_->roller_voltage(), 0);
}

TEST_F(WristTest, Disabled) {
  outputs_enabled_ = false;
  EXPECT_EQ(wrist_output_proto_->wrist_voltage(),0.0);
}

TEST_F(WristTest, Calibration) {
  outputs_enabled_ = true;
  intake_mode_ = c2018::score_subsystem::IntakeMode::INTAKE;
  wrist_input_proto_->set_wrist_hall(false);
  wrist_input_proto_->set_wrist_encoder(0);

  SetGoal();
  CalibrationSequence();

  EXPECT_TRUE(wrist_status_proto_->wrist_calibrated());
}

TEST_F(WristTest, SysIdle) {
  outputs_enabled_ = true;
  CalibrationSequence();
  Update();

  EXPECT_EQ(wrist_output_proto_->wrist_voltage(),0);
  EXPECT_EQ(wrist_output_proto_->roller_voltage(),0);
  EXPECT_EQ(wrist_status_proto_->wrist_state(), c2018::score_subsystem::SYSTEM_IDLE);
}

TEST_F(WristTest, StateDisabled) {
  outputs_enabled_ = false;
  CalibrationSequence();
  Update();

  EXPECT_EQ(wrist_output_proto_->roller_voltage(),0);
  EXPECT_EQ(wrist_output_proto_->wrist_voltage(),0);
  EXPECT_EQ(wrist_status_proto_->wrist_state(), c2018::score_subsystem::DISABLED);
}

TEST_F(WristTest, CanCapAngle) {
  outputs_enabled_ = true;
  angle_ = -5;
  CalibrationSequence();
  Update();

  SetGoal();
  EXPECT_TRUE(wrist_status_proto_->wrist_position() >=
              muan::utils::Cap(wrist_status_proto_->wrist_position(), 0, M_PI) - 0.01);
  }

TEST_F(WristTest, CanCapU) {
  outputs_enabled_ = true;
  wrist_voltage_ = -5;
  CalibrationSequence();
  Update();

  SetGoal();

  EXPECT_TRUE(wrist_output_proto_->wrist_voltage() >=
              muan::utils::Cap(wrist_output_proto_->wrist_voltage(), -12, 12) - 0.01);
}
