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
  }

  void Update(bool outputs_enabled) {
    wrist_.Update(input_, &output_, &status_, outputs_enabled);
  }

  void SetGoal(double angle) {
    goal_->set_angle(angle);
    wrist_.SetGoal(goal_);
  }

  void CalibrateDisabled() {
    input_->set_wrist_encoder(0);
    input_->set_wrist_zeroed(true);
    Update(false);
    EXPECT_TRUE(status_->is_calibrated());
    EXPECT_NEAR(status_->wrist_angle(), 0, 1e-2);
  }

  WristInputProto input_;
  WristStatusProto status_;
  WristOutputProto output_;
  WristGoalProto goal_;

 private:
  Wrist wrist_;
};

TEST_F(WristFixture, Disabled) {
  CalibrateDisabled();

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
  CalibrateDisabled();  // Setpoint should always be pi higher than actual
  SetGoal(M_PI / 2.);
  Update(true);
  UpdateInputs();
  Update(true);

  EXPECT_NEAR(output_->wrist_setpoint(), M_PI / 2., 1e-2);
  EXPECT_NEAR(status_->wrist_angle(), M_PI / 2., 1e-2);
  EXPECT_EQ(output_->output_type(), POSITION);
}

}  // namespace wrist
}  // namespace c2019
