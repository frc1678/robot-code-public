#include "c2019/subsystems/superstructure/wrist/wrist.h"
#include "gtest/gtest.h"

namespace c2019 {
namespace wrist {

class WristFixture : public ::testing::Test {
 public:
  void UpdateInputs() {
    if (output_->output_type() == POSITION) {
      input_->set_wrist_encoder(output_->wrist_setpoint());
    }
    input_->set_wrist_hall(std::abs(input_->wrist_encoder() - wrist_.offset() -
                                    kHallEffectAngle) < 1e-3);
  }

  void Update(bool outputs_enabled) {
    wrist_.Update(input_, &output_, &status_, outputs_enabled);
  }

  void SetGoal(double angle) {
    goal_->set_angle(angle);
    wrist_.SetGoal(goal_);
  }

  void CalibrateDisabled(double offset = 0) {
    input_->set_wrist_encoder(offset);
    input_->set_wrist_hall(false);
    Update(false);
    for (int i = 0; i < 2500; i++) {
      input_->set_wrist_encoder(offset + i * (M_PI / 2500.));
      input_->set_wrist_hall(
          std::abs(input_->wrist_encoder() - offset - kHallEffectAngle) < 1e-3);
      Update(false);
    }
    EXPECT_TRUE(wrist_.is_calibrated());
    EXPECT_NEAR(status_->wrist_angle(), M_PI, 1e-2);
    EXPECT_NEAR(-wrist_.offset(), offset, 1e-2);
  }

  WristInputProto input_;
  WristStatusProto status_;
  WristOutputProto output_;
  WristGoalProto goal_;

 private:
  Wrist wrist_;
};

TEST_F(WristFixture, Disabled) {
  CalibrateDisabled(M_PI);

  SetGoal(M_PI);
  Update(false);
  UpdateInputs();

  EXPECT_EQ(output_->wrist_setpoint(), 0);
  EXPECT_EQ(output_->output_type(), OPEN_LOOP);
}

TEST_F(WristFixture, NotCalibrated) {
  SetGoal(M_PI);
  Update(true);
  UpdateInputs();
  Update(true);

  EXPECT_EQ(output_->wrist_setpoint(), 0);
  EXPECT_EQ(output_->output_type(), OPEN_LOOP);
}

TEST_F(WristFixture, Offsets) {
  CalibrateDisabled(M_PI);  // Setpoint should always be pi higher than actual
  SetGoal(M_PI / 2.);
  Update(true);
  UpdateInputs();
  Update(true);

  EXPECT_NEAR(output_->wrist_setpoint(), M_PI / 2. + M_PI, 1e-2);
  EXPECT_NEAR(status_->wrist_angle(), M_PI / 2., 1e-2);
  EXPECT_EQ(output_->output_type(), POSITION);
}

}  // namespace wrist
}  // namespace c2019
