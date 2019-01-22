#include "c2019/subsystems/superstructure/elevator/elevator.h"
#include "gtest/gtest.h"

namespace c2019 {
namespace elevator {

class ElevatorFixture : public ::testing::Test {
 public:
  void SetGoal(double height, bool high_gear, bool crawler_down, bool crawling,
               bool brake) {
    goal_->set_height(height);
    goal_->set_crawling(crawling);
    goal_->set_crawler_down(crawler_down);
    goal_->set_high_gear(high_gear);
    goal_->set_brake(brake);

    elevator_.SetGoal(goal_);
  }

  void Update(bool outputs_enabled, bool fix_inputs, bool calibrated) {
    if (fix_inputs) {
      if (output_->elevator_output_type() == POSITION) {
        input_->set_elevator_encoder(output_->elevator_setpoint());
      }

      input_->set_elevator_velocity(0);
      input_->set_elevator_hall(false);
      input_->set_zeroed(calibrated);
      input_->set_has_hatch(false);
      input_->set_has_cargo(false);
      input_->set_elevator_voltage(0);
      input_->set_elevator_current(0);
    }

    elevator_.Update(input_, &output_, &status_, outputs_enabled);
  }

 private:
  Elevator elevator_;

 protected:
  ElevatorGoalProto goal_;
  ElevatorStatusProto status_;
  ElevatorInputProto input_;
  ElevatorOutputProto output_;
};

TEST_F(ElevatorFixture, Sanity) {
  SetGoal(0, false, false, false, false);
  Update(true, true, true);
  Update(true, true, true);

  EXPECT_EQ(0, output_->elevator_setpoint());
  EXPECT_EQ(output_->elevator_ff(), kFF);
  EXPECT_EQ(output_->elevator_output_type(), POSITION);
}

TEST_F(ElevatorFixture, Disabled) {
  SetGoal(0, false, false, false, false);
  Update(false, true, true);
  Update(false, true, true);

  EXPECT_EQ(0, output_->elevator_setpoint());
  EXPECT_EQ(output_->elevator_ff(), 0);
  EXPECT_EQ(output_->elevator_output_type(), OPEN_LOOP);
}

TEST_F(ElevatorFixture, NotCalibrated) {
  SetGoal(0, false, false, false, false);
  Update(true, true, false);
  Update(true, true, false);

  EXPECT_EQ(0, output_->elevator_setpoint());
  EXPECT_EQ(output_->elevator_ff(), 0);
  EXPECT_EQ(output_->elevator_output_type(), OPEN_LOOP);
}

TEST_F(ElevatorFixture, Heights) {
  double elevator_goal = 0;
  SetGoal(elevator_goal, false, false, false, false);
  Update(true, true, true);
  Update(true, true, true);

  EXPECT_EQ(elevator_goal, output_->elevator_setpoint());
  EXPECT_EQ(output_->elevator_ff(), kFF);
  EXPECT_EQ(output_->elevator_output_type(), POSITION);

  elevator_goal = 1;
  SetGoal(elevator_goal, false, false, false, false);
  Update(true, true, true);
  Update(true, true, true);

  EXPECT_EQ(elevator_goal, output_->elevator_setpoint());
  EXPECT_EQ(output_->elevator_ff(), kFF);
  EXPECT_EQ(output_->elevator_output_type(), POSITION);

  elevator_goal = 1.5;
  SetGoal(elevator_goal, false, false, false, false);
  Update(true, true, true);
  Update(true, true, true);

  EXPECT_EQ(elevator_goal, output_->elevator_setpoint());
  EXPECT_EQ(output_->elevator_ff(), kFF + kFFSecondStage);
  EXPECT_EQ(output_->elevator_output_type(), POSITION);
}

TEST_F(ElevatorFixture, CapGoal) {
  double elevator_goal = 1e10;
  SetGoal(elevator_goal, false, false, false, false);
  Update(true, true, true);
  Update(true, true, true);

  EXPECT_EQ(kMaxHeight, output_->elevator_setpoint());
  EXPECT_EQ(output_->elevator_ff(), kFF + kFFSecondStage);
  EXPECT_EQ(output_->elevator_output_type(), POSITION);
}

TEST_F(ElevatorFixture, Brake) {
  // Test holding position at ~0.5m
  double elevator_goal = 0.5;
  SetGoal(elevator_goal, false, false, false, false);
  Update(true, true, true);
  Update(true, true, true);

  EXPECT_EQ(elevator_goal, output_->elevator_setpoint());
  EXPECT_EQ(output_->elevator_ff(), kFF);
  EXPECT_EQ(output_->elevator_output_type(), POSITION);
  EXPECT_EQ(input_->elevator_encoder(), elevator_goal);

  aos::time::EnableMockTime(aos::monotonic_clock::now());

  // Apply brake
  SetGoal(elevator_goal, false, false, false, true);
  Update(true, true, true);
  Update(true, true, true);
  EXPECT_EQ(elevator_goal, output_->elevator_setpoint());
  EXPECT_EQ(output_->elevator_ff(), kFF);
  EXPECT_EQ(output_->elevator_output_type(), POSITION);
  EXPECT_EQ(input_->elevator_encoder(), elevator_goal);
  EXPECT_FALSE(status_->braked());
  EXPECT_TRUE(output_->brake());

  // Time will pass for brake to apply
  aos::time::IncrementMockTime(std::chrono::milliseconds(210));
  Update(true, true, true);
  Update(true, true, true);
  EXPECT_EQ(0, output_->elevator_setpoint());
  EXPECT_EQ(output_->elevator_ff(), 0);
  EXPECT_EQ(output_->elevator_output_type(), OPEN_LOOP);
  EXPECT_EQ(input_->elevator_encoder(), elevator_goal);
  EXPECT_TRUE(status_->braked());
  EXPECT_TRUE(output_->brake());
}

}  // namespace elevator
}  // namespace c2019
