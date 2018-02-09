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
    wrist_input_proto_->set_wrist_hall(plant_.x(0) >= 0.21 &&
                                       plant_.x(0) <= 0.25);
    wrist_.Update(wrist_input_proto_, &wrist_output_proto_,
                  &wrist_status_proto_, outputs_enabled_);
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
    wrist_input_proto_->set_wrist_encoder(plant_.y(0) + offset);
    Update();
    EXPECT_NEAR(wrist_output_proto_->wrist_voltage(), 0, 12);
  }

  EXPECT_NEAR(wrist_status_proto_->wrist_angle(), M_PI / 4, 0.01);
  EXPECT_TRUE(wrist_status_proto_->wrist_calibrated());
}

TEST_F(WristTest, Intake) {
  wrist_input_proto_->set_wrist_encoder(0);
  wrist_input_proto_->set_wrist_hall(false);
  outputs_enabled_ = true;
  SetGoal(0.0, c2018::score_subsystem::IntakeMode::INTAKE);
  Update();

  EXPECT_NEAR(wrist_output_proto_->intake_voltage(), 12.0, 1e-3);
  EXPECT_FALSE(wrist_output_proto_->wrist_solenoid_open());
  EXPECT_FALSE(wrist_output_proto_->wrist_solenoid_close());
}

TEST_F(WristTest, Outtake) {
  wrist_input_proto_->set_wrist_encoder(0);
  wrist_input_proto_->set_wrist_hall(false);
  outputs_enabled_ = true;
  SetGoal(0.0, c2018::score_subsystem::IntakeMode::OUTTAKE);
  Update();

  EXPECT_NEAR(wrist_output_proto_->intake_voltage(), -12.0, 1e-3);
  EXPECT_FALSE(wrist_output_proto_->wrist_solenoid_open());
  EXPECT_FALSE(wrist_output_proto_->wrist_solenoid_close());
}

TEST_F(WristTest, Idle) {
  wrist_input_proto_->set_wrist_encoder(0);
  wrist_input_proto_->set_wrist_hall(false);
  outputs_enabled_ = true;
  SetGoal(0.0, c2018::score_subsystem::IntakeMode::IDLE);
  Update();

  EXPECT_NEAR(wrist_output_proto_->intake_voltage(), 0, 1e-3);
  EXPECT_FALSE(wrist_output_proto_->wrist_solenoid_open());
  EXPECT_TRUE(wrist_output_proto_->wrist_solenoid_close());
}

TEST_F(WristTest, Stow) {
  wrist_input_proto_->set_wrist_encoder(0);
  wrist_input_proto_->set_wrist_hall(false);
  outputs_enabled_ = true;
  SetGoal(c2018::score_subsystem::wrist::kWristStowAngle,
          c2018::score_subsystem::IntakeMode::IDLE);
  Update();

  for (int i = 0; i < 1000; i++) {
    wrist_input_proto_->set_wrist_encoder(plant_.y(0));
    Update();
    EXPECT_NEAR(wrist_output_proto_->wrist_voltage(), 0, 12);
  }

  EXPECT_NEAR(wrist_status_proto_->wrist_angle(),
              c2018::score_subsystem::wrist::kWristStowAngle, 0.01);
  EXPECT_TRUE(wrist_status_proto_->wrist_calibrated());

  EXPECT_NEAR(wrist_output_proto_->intake_voltage(), 0, 1e-3);
  EXPECT_FALSE(wrist_output_proto_->wrist_solenoid_open());
  EXPECT_TRUE(wrist_output_proto_->wrist_solenoid_close());
  EXPECT_NEAR(wrist_status_proto_->wrist_profiled_goal(),
              c2018::score_subsystem::wrist::kWristStowAngle, 1e-3);
  EXPECT_NEAR(wrist_status_proto_->wrist_unprofiled_goal(),
              c2018::score_subsystem::wrist::kWristStowAngle, 1e-3);
}

TEST_F(WristTest, Backwards) {
  wrist_input_proto_->set_wrist_encoder(0);
  wrist_input_proto_->set_wrist_hall(false);
  outputs_enabled_ = true;
  SetGoal(c2018::score_subsystem::wrist::kWristMaxAngle,
          c2018::score_subsystem::IntakeMode::IDLE);
  Update();

  for (int i = 0; i < 1000; i++) {
    wrist_input_proto_->set_wrist_encoder(plant_.y(0));
    Update();
    EXPECT_NEAR(wrist_output_proto_->wrist_voltage(), 0, 12);
  }

  EXPECT_NEAR(wrist_status_proto_->wrist_angle(),
              c2018::score_subsystem::wrist::kWristMaxAngle, 1e-3);
  EXPECT_TRUE(wrist_status_proto_->wrist_calibrated());
  EXPECT_NEAR(wrist_status_proto_->wrist_profiled_goal(),
              c2018::score_subsystem::wrist::kWristMaxAngle, 1e-3);
  EXPECT_NEAR(wrist_status_proto_->wrist_unprofiled_goal(),
              c2018::score_subsystem::wrist::kWristMaxAngle, 1e-3);

  EXPECT_NEAR(wrist_output_proto_->intake_voltage(), 0, 1e-3);
  EXPECT_FALSE(wrist_output_proto_->wrist_solenoid_open());
  EXPECT_TRUE(wrist_output_proto_->wrist_solenoid_close());
}

TEST_F(WristTest, Disabled) {
  wrist_input_proto_->set_wrist_encoder(0);
  wrist_input_proto_->set_wrist_hall(false);
  outputs_enabled_ = false;

  SetGoal(c2018::score_subsystem::wrist::kWristMaxAngle,
          c2018::score_subsystem::IntakeMode::INTAKE);

  Update();

  EXPECT_EQ(wrist_output_proto_->wrist_voltage(), 0.0);
  EXPECT_EQ(wrist_output_proto_->intake_voltage(), 0.0);
}

TEST_F(WristTest, CanCapAngle) {
  outputs_enabled_ = true;
  SetGoal(6e5, c2018::score_subsystem::IntakeMode::IDLE);

  Update();

  EXPECT_TRUE(wrist_status_proto_->wrist_profiled_goal() <=
              c2018::score_subsystem::wrist::kWristMaxAngle);
  EXPECT_TRUE(wrist_status_proto_->wrist_unprofiled_goal() >=
              c2018::score_subsystem::wrist::kWristMinAngle);
  EXPECT_TRUE(wrist_status_proto_->wrist_profiled_goal() <=
              c2018::score_subsystem::wrist::kWristMaxAngle);
  EXPECT_TRUE(wrist_status_proto_->wrist_unprofiled_goal() >=
              c2018::score_subsystem::wrist::kWristMinAngle);
}
