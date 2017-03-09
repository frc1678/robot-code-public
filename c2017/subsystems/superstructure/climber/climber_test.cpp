#include "gtest/gtest.h"
#include "c2017/subsystems/superstructure/climber/climber.h"
#include "c2017/queue_manager/queue_manager.h"

class ClimberTest : public ::testing::Test {
 public:
  ClimberTest() {}

  void Update(double current, double position, bool robot_disabled, int num_ticks) {
    for (int i = 0; i < num_ticks; i++) {
      input->set_current(current);
      input->set_position(position);
      output = climber.Update(input, !robot_disabled);
      current_velocity = (position - last_position) / 0.005;
      last_position = position;
    }
  }

  double GetVoltage() { return output->voltage(); }

  double GetVelocity() { return current_velocity; }

  void SetGoal(bool should_climb) {
    goal->set_climbing(should_climb);
    climber.SetGoal(goal);
  }

  c2017::climber::State GetClimberState() { return climber.current_state(); }

 private:
  c2017::climber::Climber climber;
  c2017::climber::ClimberInputProto input;
  c2017::climber::ClimberGoalProto goal;
  c2017::climber::ClimberOutputProto output;
  double last_position = 0;
  double current_velocity = 0;
};

TEST_F(ClimberTest, DoNothing) {
  SetGoal(false);
  Update(0, 0, false, 1);

  EXPECT_EQ(GetClimberState(), c2017::climber::NOTHING);
  EXPECT_EQ(GetVoltage(), 0.0);
}

TEST_F(ClimberTest, Approaching) {
  SetGoal(true);

  for (double i = 0; i < 500; (i = i + 0.01)) {
    Update(0, i, false, 1);
  }


  EXPECT_EQ(GetClimberState(), c2017::climber::APPROACHING);
  EXPECT_EQ(GetVoltage(), 9.0);
}

TEST_F(ClimberTest, AtTop) {
  SetGoal(true);

  for (double i = 0; i < 500; (i = i + 0.01)) {
    Update(0, i, false, 1);
  }
  EXPECT_EQ(GetClimberState(), c2017::climber::APPROACHING);

  for (int i = 0; i < 500; i += 2) {
    Update(0, i, false, 1);
  }
  EXPECT_EQ(GetClimberState(), c2017::climber::CLIMBING);

  for (int i = 5000; i > 0; i -= 200) {
    Update(0, i, false, 1);
  }

  EXPECT_EQ(GetClimberState(), c2017::climber::AT_TOP);
  EXPECT_EQ(GetVoltage(), 0.0);
}
