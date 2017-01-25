#include <iostream>
#include "gtest/gtest.h"
#include "c2017/subsystems/superstructure/climber/climber.h"
#include "c2017/queue_manager/queue_manager.h"

class ClimberTest : public ::testing::Test {
 public:
  ClimberTest() {}

  void Update(double voltage, bool on_rope) {
     if (on_rope) {
       current_position_ += (current_position_ - initial_rope_position) > 1 ? 0 : 0.00042 * voltage;
       current_ = current_position_ > 2 ? 120 : 20;
     } else {
       current_position_ += 0.00042 * voltage;
       current_ = current_position_ > 2 ? 120 : 10;
       initial_rope_position = current_position_;
     }
  }
  double GetPosition() {
    return current_position_;
  }

  double GetCurrent() {
    return current_;
  }

  void Reset() {
    current_position_ = 0;
    current_ = 0;
    initial_rope_position = 0;
  }

 private:
  double initial_rope_position = 0;
  double current_position_ = 0;
  double current_ = 0;
};

TEST_F(ClimberTest, ClimbsToTheTop) {
  c2017::climber::ClimberGoalProto goal;
  c2017::climber::ClimberInputProto input;
  c2017::climber::ClimberOutputProto output;

  muan::wpilib::DriverStationProto ds_status;
  ds_status->set_mode(RobotMode::TELEOP);

  goal->set_climbing(true);

  c2017::climber::Climber test_climber;

  test_climber.SetGoal(goal);

  auto test_status = c2017::QueueManager::GetInstance().climber_status_queue().ReadLastMessage();

  for (double t = 0; t < 5; t += 0.005) {
    input->set_position(GetPosition());
    input->set_current(GetCurrent());
    output = test_climber.Update(input, ds_status);
    test_status = c2017::QueueManager::GetInstance().climber_status_queue().ReadLastMessage();
    if (test_status) {
      Update(output->voltage(), test_status.value()->on_rope());
    }
  }
  if (test_status) {
    EXPECT_TRUE(test_status.value()->currently_climbing());
    EXPECT_TRUE(test_status.value()->hit_top());
  }
  EXPECT_NEAR(output->voltage(), 0, 1e-5);
}

TEST_F(ClimberTest, Disabled) {
  c2017::climber::ClimberGoalProto goal;
  c2017::climber::ClimberInputProto input;
  c2017::climber::ClimberOutputProto output;

  muan::wpilib::DriverStationProto ds_status;
  ds_status->set_mode(RobotMode::DISABLED);

  goal->set_climbing(true);
  input->set_position(GetPosition());  // position helps find the rate of the encoder.

  c2017::climber::Climber test_climber;

  test_climber.SetGoal(goal);

  auto test_status = c2017::QueueManager::GetInstance().climber_status_queue().ReadLastMessage();

  for (double t = 0; t < 2; t += 0.005) {
    input->set_position(GetPosition());
    input->set_current(GetCurrent());
    output = test_climber.Update(input, ds_status);
    test_status = c2017::QueueManager::GetInstance().climber_status_queue().ReadLastMessage();
    if (test_status) {
      Update(output->voltage(), test_status.value()->on_rope());
    }
    if (GetPosition() < 1) {
      EXPECT_NEAR(output->voltage(), 0, 1e-5);
    }
  }


    if (test_status) {
    EXPECT_FALSE(test_status.value()->currently_climbing());
    EXPECT_FALSE(test_status.value()->hit_top());
    }
  EXPECT_NEAR(output->voltage(), 0, 1e-5);
}
