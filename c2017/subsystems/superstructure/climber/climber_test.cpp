#include "gtest/gtest.h"
#include "climber.h"
#include <iostream>
#include "c2017/queue_manager/queue_manager.h"

class ClimberTest : public ::testing::Test {
 public:
  ClimberTest() {}
  void Update(double voltage) {
     current_position_ += current_position_ > 1 ? 1 : 0.00042 * voltage;
     current_ = voltage / 10;
  }
  double GetPosition() {
    return current_position_;
  }

  void Reset() {
    current_position_ = 0;
    current_ = 0;
  }

 private:
  double current_position_ = 0;
  double current_ = 0;
};

TEST_F(ClimberTest, ClimbsToTheTop) {
  c2017::climber::ClimberGoalProto goal;
  c2017::climber::ClimberInputProto input;

  muan::wpilib::DriverStationProto ds_status;
  ds_status->set_mode(RobotMode::TELEOP);

  goal->set_climbing(true);
  input->set_position(GetPosition());  // position helps find the rate of the encoder. Encoder has been running throughout the match as the shooter so its big.

  c2017::climber::Climber test_climber;

  test_climber.SetGoal(goal);

  auto output = test_climber.Update(input, ds_status);
  
  for (double t = -1.005; t < 2; t += 0.005) {
    input->set_position(GetPosition());
    if (GetPosition() < 1) {
      EXPECT_NEAR(output->voltage(), 12, 1e-5);
    }
    Update(output->voltage());
  }

  auto test_status = c2017::QueueManager::GetInstance().climber_status_queue().ReadLastMessage();

  if(test_status) {
  EXPECT_TRUE(test_status.value()->currently_climbing());
  EXPECT_TRUE(test_status.value()->hit_top());
  }
  EXPECT_NEAR(output->voltage(), 0, 1e-5);
}

TEST_F(ClimberTest, Disabled) {
  c2017::climber::ClimberGoalProto goal;
  c2017::climber::ClimberInputProto input;
  
  muan::wpilib::DriverStationProto ds_status;
  ds_status->set_mode(RobotMode::DISABLED);
  
  goal->set_climbing(true);
  input->set_position(GetPosition());  // position helps find the rate of the encoder.
  
  c2017::climber::Climber test_climber;

  test_climber.SetGoal(goal);
  
  auto output = test_climber.Update(input, ds_status);
  for (double t = 0; t < 2; t += 0.005) {
    input->set_position(GetPosition());
    if (GetPosition() < 1) {
      EXPECT_NEAR(output->voltage(), 0, 1e-5);
    }
    auto test_status = c2017::QueueManager::GetInstance().climber_status_queue().ReadLastMessage();

    if(test_status) {
    EXPECT_FALSE(test_status.value()->currently_climbing());
    EXPECT_FALSE(test_status.value()->hit_top());
    }
    EXPECT_NEAR(output->voltage(), 0, 1e-5);
  }
}
