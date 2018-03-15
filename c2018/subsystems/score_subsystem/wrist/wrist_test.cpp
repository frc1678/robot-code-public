#include "c2018/subsystems/score_subsystem/wrist/wrist.h"
#include "gtest/gtest.h"

namespace c2018 {

namespace score_subsystem {

namespace wrist {

class WristTest : public ::testing::Test {
 public:
  WristTest() {
    plant_ = muan::control::StateSpacePlant<1, 3, 1>(
        frc1678::wrist::controller::A(),
        frc1678::wrist::controller::B(),
        frc1678::wrist::controller::C());
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

  void SetGoal(double angle, IntakeMode intake_mode, bool is_open = false) {
    wrist_.SetGoal(angle, intake_mode, is_open);
  }

  ScoreSubsystemInputProto wrist_input_proto_;
  ScoreSubsystemOutputProto wrist_output_proto_;
  ScoreSubsystemStatusProto wrist_status_proto_;
  ScoreSubsystemGoalProto wrist_goal_proto_;

  double wrist_voltage_;
  bool outputs_enabled_;

 protected:
  muan::control::StateSpacePlant<1, 3, 1> plant_;

 private:
  WristController wrist_;
};

TEST_F(WristTest, Calib) {
  wrist_input_proto_->set_wrist_encoder(0);
  wrist_input_proto_->set_wrist_hall(false);
  outputs_enabled_ = true;

  SetGoal(M_PI / 4, IntakeMode::IDLE);

  double offset = -M_PI / 2;

  for (int i = 0; i < 1000; i++) {
    wrist_input_proto_->set_wrist_encoder(plant_.y(0) + offset);
    Update();
    EXPECT_NEAR(wrist_output_proto_->wrist_voltage(), 0, 12);
  }

  EXPECT_NEAR(wrist_status_proto_->wrist_angle(), M_PI / 4, 1e-3);
  EXPECT_TRUE(wrist_status_proto_->wrist_calibrated());
}

TEST_F(WristTest, Intake) {
  wrist_input_proto_->set_wrist_encoder(0);
  wrist_input_proto_->set_wrist_hall(false);
  outputs_enabled_ = true;
  SetGoal(0.0, IntakeMode::IN);
  Update();

  EXPECT_NEAR(wrist_output_proto_->intake_voltage(), 12.0, 1e-3);
  EXPECT_FALSE(wrist_output_proto_->wrist_solenoid_open());
  EXPECT_FALSE(wrist_output_proto_->wrist_solenoid_close());
}

TEST_F(WristTest, Outtake) {
  wrist_input_proto_->set_wrist_encoder(0);
  wrist_input_proto_->set_wrist_hall(false);
  outputs_enabled_ = true;
  SetGoal(0.0, IntakeMode::OUT_FAST);
  Update();

  EXPECT_NEAR(wrist_output_proto_->intake_voltage(), kFastOuttakeVoltage, 1e-3);
  EXPECT_FALSE(wrist_output_proto_->wrist_solenoid_open());
  EXPECT_TRUE(wrist_output_proto_->wrist_solenoid_close());
}

TEST_F(WristTest, Idle) {
  wrist_input_proto_->set_wrist_encoder(0);
  wrist_input_proto_->set_wrist_hall(false);
  outputs_enabled_ = true;
  SetGoal(0.0, IntakeMode::IDLE);
  Update();

  EXPECT_NEAR(wrist_output_proto_->intake_voltage(), 0, 1e-3);
  EXPECT_FALSE(wrist_output_proto_->wrist_solenoid_open());
  EXPECT_TRUE(wrist_output_proto_->wrist_solenoid_close());
}

TEST_F(WristTest, Stow) {
  wrist_input_proto_->set_wrist_encoder(0);
  wrist_input_proto_->set_wrist_hall(false);
  outputs_enabled_ = true;
  SetGoal(kWristStowAngle, IntakeMode::IDLE);
  Update();

  for (int i = 0; i < 1000; i++) {
    wrist_input_proto_->set_wrist_encoder(plant_.y(0));
    Update();
    EXPECT_NEAR(wrist_output_proto_->wrist_voltage(), 0, 12);
  }

  EXPECT_NEAR(wrist_status_proto_->wrist_angle(), kWristStowAngle, 1e-3);
  EXPECT_TRUE(wrist_status_proto_->wrist_calibrated());

  EXPECT_NEAR(wrist_output_proto_->intake_voltage(), 0, 1e-3);
  EXPECT_FALSE(wrist_output_proto_->wrist_solenoid_open());
  EXPECT_TRUE(wrist_output_proto_->wrist_solenoid_close());
  EXPECT_NEAR(wrist_status_proto_->wrist_profiled_goal(), kWristStowAngle,
              1e-3);
  EXPECT_NEAR(wrist_status_proto_->wrist_unprofiled_goal(), kWristStowAngle,
              1e-3);
}

TEST_F(WristTest, Backwards) {
  wrist_input_proto_->set_wrist_encoder(0);
  wrist_input_proto_->set_wrist_hall(false);
  outputs_enabled_ = true;
  SetGoal(kWristMaxAngle, IntakeMode::IDLE);
  Update();

  for (int i = 0; i < 1000; i++) {
    wrist_input_proto_->set_wrist_encoder(plant_.y(0));
    Update();
    EXPECT_NEAR(wrist_output_proto_->wrist_voltage(), 0, 12);
  }

  EXPECT_NEAR(wrist_status_proto_->wrist_angle(), kWristMaxAngle, 1e-3);
  EXPECT_TRUE(wrist_status_proto_->wrist_calibrated());
  EXPECT_NEAR(wrist_status_proto_->wrist_profiled_goal(), kWristMaxAngle, 1e-3);
  EXPECT_NEAR(wrist_status_proto_->wrist_unprofiled_goal(), kWristMaxAngle,
              1e-3);

  EXPECT_NEAR(wrist_output_proto_->intake_voltage(), 0, 1e-3);
  EXPECT_FALSE(wrist_output_proto_->wrist_solenoid_open());
  EXPECT_TRUE(wrist_output_proto_->wrist_solenoid_close());
}

TEST_F(WristTest, Disabled) {
  wrist_input_proto_->set_wrist_encoder(0);
  wrist_input_proto_->set_wrist_hall(false);
  outputs_enabled_ = false;

  SetGoal(kWristMaxAngle, IntakeMode::IN);

  Update();

  EXPECT_EQ(wrist_output_proto_->wrist_voltage(), 0.0);
  EXPECT_EQ(wrist_output_proto_->intake_voltage(), 0.0);
}

TEST_F(WristTest, CanCapAngle) {
  outputs_enabled_ = true;
  SetGoal(6e5, IntakeMode::IDLE);

  Update();

  EXPECT_TRUE(wrist_status_proto_->wrist_profiled_goal() <= kWristMaxAngle);
  EXPECT_TRUE(wrist_status_proto_->wrist_unprofiled_goal() >= kWristMinAngle);
  EXPECT_TRUE(wrist_status_proto_->wrist_profiled_goal() <= kWristMaxAngle);
  EXPECT_TRUE(wrist_status_proto_->wrist_unprofiled_goal() >= kWristMinAngle);
}

}  // namespace wrist

}  // namespace score_subsystem

}  // namespace c2018
