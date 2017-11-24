#include "o2017/subsystems/superstructure/climber/climber.h"
#include "gtest/gtest.h"
#include "o2017/queue_manager/queue_manager.h"

namespace o2017 {

namespace superstructure {

namespace tests {

class ClimberTest : public ::testing::Test {
 public:
  ClimberTest() {}

  void Update(double current, double position, bool robot_disabled) {
    input->set_climber_current(current);
    input->set_climber_position(position);
    climber.Update(input, goal, &output, &status, !robot_disabled);
    last_position = position;
  }

  double GetVoltage() { return output->climber_voltage(); }

  void SetGoal(bool should_climb) {
    goal->set_should_climb(should_climb);
    climber.SetGoal(goal);
  }

  ClimberState GetClimberState() { return climber.current_state(); }

 private:
  climber::Climber climber;
  SuperstructureInputProto input;
  SuperstructureGoalProto goal;
  SuperstructureOutputProto output;
  SuperstructureStatusProto status;
  double last_position = 0;
};

TEST_F(ClimberTest, DoNothing) {
  SetGoal(false);
  Update(0, 0, false);

  EXPECT_EQ(GetClimberState(), NOTHING);
  EXPECT_EQ(GetVoltage(), 0.0);
}

TEST_F(ClimberTest, SpinningUp) {
  SetGoal(true);

  for (double i = 0; i < 500; (i += 0.001)) {
    Update(0, i, false);
  }

  EXPECT_EQ(GetClimberState(), SPIN_UP);
  EXPECT_EQ(GetVoltage(), o2017::superstructure::climber::kClimbingVoltage);
}

TEST_F(ClimberTest, Approaching) {
  SetGoal(true);

  double d = 0.0;

  for (; d < 500; (d += 0.001)) {
    Update(0, d, false);
  }
  EXPECT_EQ(GetClimberState(), SPIN_UP);

  for (; d < 1000; (d += 0.01)) {
    Update(0, d, false);
  }

  EXPECT_EQ(GetClimberState(), APPROACHING);
  EXPECT_EQ(GetVoltage(), o2017::superstructure::climber::kClimbingVoltage);
}

TEST_F(ClimberTest, ReachedTop) {
  SetGoal(true);
  double position = 0.0;

  for (int i = 0; i < 200; i++) {
    position += 0.001;
    Update(0, position, false);
  }
  EXPECT_EQ(GetClimberState(), SPIN_UP);

  for (int i = 0; i < 200; i++) {
    position += 0.01;
    Update(0, position, false);
  }
  EXPECT_EQ(GetClimberState(), APPROACHING);

  for (int i = 0; i < 200; i++) {
    position += 0.002;
    Update(0, position, false);
  }
  EXPECT_EQ(GetClimberState(), CLIMBING);

  for (int i = 0; i < 200; i++) {
    Update(0, position, false);
  }

  EXPECT_EQ(GetClimberState(), REACHED_TOP);
  EXPECT_EQ(GetVoltage(), o2017::superstructure::climber::kTopVoltage);
}

}  // namespace tests

}  // namespace superstructure

}  // namespace o2017
