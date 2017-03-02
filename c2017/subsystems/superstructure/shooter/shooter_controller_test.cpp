#include "c2017/subsystems/superstructure/shooter/shooter_controller.h"
#include "c2017/subsystems/superstructure/shooter/queue_types.h"
#include "c2017/subsystems/superstructure/shooter/shooter.pb.h"
#include "gtest/gtest.h"
#include "muan/wpilib/queue_types.h"

TEST(ShooterControllerTest, IsSane) {
  auto plant = muan::control::StateSpacePlant<1, 3, 1>(frc1678::shooter_controller::controller::A(),
                                                       frc1678::shooter_controller::controller::B(),
                                                       frc1678::shooter_controller::controller::C());

  plant.x(0) = 0.0;
  plant.x(1) = 0.0;
  plant.x(2) = 0.0;

  plant.Update((Eigen::Matrix<double, 1, 1>() << 0.).finished());

  EXPECT_NEAR(plant.x(0), 0, 1e-5);
  EXPECT_NEAR(plant.x(1), 0, 1e-5);
  EXPECT_NEAR(plant.x(2), 0, 1e-5);
}

TEST(ShooterControllerTest, PositiveVelocity) {
  c2017::shooter::ShooterInputProto input;
  c2017::shooter::ShooterOutputProto output;
  c2017::shooter::ShooterGoalProto goal;

  c2017::shooter::ShooterController shooter_;

  auto plant = muan::control::StateSpacePlant<1, 3, 1>(frc1678::shooter_controller::controller::A(),
                                                       frc1678::shooter_controller::controller::B(),
                                                       frc1678::shooter_controller::controller::C());

  plant.x(0) = 0.0;
  plant.x(1) = 0.0;
  plant.x(2) = 0.0;

  for (int i = 0; i <= 1e3; i++) {
    input->set_encoder_position(plant.x(0));

    goal->set_goal_velocity(300.0);
    goal->set_goal_mode(c2017::shooter::FENDER);

    shooter_.SetGoal(goal);

    output = shooter_.Update(input, true);

    plant.Update((Eigen::Matrix<double, 1, 1>() << output->voltage()).finished());

    EXPECT_NEAR(output->voltage(), 0., 12.);
    EXPECT_FALSE(output->hood_solenoid());
  }

  auto status = c2017::QueueManager::GetInstance().shooter_status_queue().ReadLastMessage();

  if (status) {
    EXPECT_NEAR(status.value()->observed_velocity(), 300, 10);
  } else {
    FAIL();
  }
}

TEST(ShooterControllerTest, CantTakeNegativeVoltage) {
  c2017::shooter::ShooterInputProto input;
  c2017::shooter::ShooterOutputProto output;
  c2017::shooter::ShooterGoalProto goal;

  auto status = c2017::QueueManager::GetInstance().shooter_status_queue().ReadLastMessage();

  c2017::shooter::ShooterController shooter_;

  auto plant = muan::control::StateSpacePlant<1, 3, 1>(frc1678::shooter_controller::controller::A(),
                                                       frc1678::shooter_controller::controller::B(),
                                                       frc1678::shooter_controller::controller::C());

  plant.x(0) = 0.0;
  plant.x(1) = 0.0;
  plant.x(2) = 0.0;

  for (int i = 0; i <= 1e3; i++) {
    input->set_encoder_position(plant.x(0));

    goal->set_goal_velocity(-300.0);
    goal->set_goal_mode(c2017::shooter::FENDER);

    shooter_.SetGoal(goal);

    output = shooter_.Update(input, true);
    status = c2017::QueueManager::GetInstance().shooter_status_queue().ReadLastMessage();

    plant.Update((Eigen::Matrix<double, 1, 1>() << output->voltage()).finished());

    EXPECT_NEAR(output->voltage(), 0., 12.);
    EXPECT_FALSE(output->hood_solenoid());
  }

  EXPECT_EQ(output->voltage(), 0);
  if (status) {
    EXPECT_NEAR(status.value()->observed_velocity(), 0, 1e-5);
  } else {
    FAIL();
  }
}

TEST(ShooterControllerTest, CanStop) {
  c2017::shooter::ShooterInputProto input;
  c2017::shooter::ShooterOutputProto output;
  c2017::shooter::ShooterGoalProto goal;

  c2017::shooter::ShooterController shooter_;

  auto plant = muan::control::StateSpacePlant<1, 3, 1>(frc1678::shooter_controller::controller::A(),
                                                       frc1678::shooter_controller::controller::B(),
                                                       frc1678::shooter_controller::controller::C());

  plant.x(0) = 0.0;
  plant.x(1) = 0.0;
  plant.x(2) = 0.0;

  auto status = c2017::QueueManager::GetInstance().shooter_status_queue().ReadLastMessage();

  for (int i = 0; i <= 1e3; i++) {
    input->set_encoder_position(plant.x(0));

    goal->set_goal_velocity(0.0);
    goal->set_goal_mode(c2017::shooter::FENDER);

    shooter_.SetGoal(goal);

    output = shooter_.Update(input, true);
    status = c2017::QueueManager::GetInstance().shooter_status_queue().ReadLastMessage();

    plant.Update((Eigen::Matrix<double, 1, 1>() << output->voltage()).finished());

    EXPECT_NEAR(output->voltage(), 0., 12.);
    EXPECT_FALSE(output->hood_solenoid());
  }

  if (status) {
    EXPECT_NEAR(status.value()->observed_velocity(), 0, 1e-5);
  } else {
    FAIL();
  }
}

TEST(ShooterControllerTest, FenderMode) {
  c2017::shooter::ShooterInputProto input;
  c2017::shooter::ShooterOutputProto output;
  c2017::shooter::ShooterGoalProto goal;

  c2017::shooter::ShooterController shooter_;

  auto plant = muan::control::StateSpacePlant<1, 3, 1>(frc1678::shooter_controller::controller::A(),
                                                       frc1678::shooter_controller::controller::B(),
                                                       frc1678::shooter_controller::controller::C());

  plant.x(0) = 0.0;
  plant.x(1) = 0.0;
  plant.x(2) = 0.0;

  goal->set_goal_mode(c2017::shooter::ShotMode::FENDER);

  shooter_.SetGoal(goal);

  output = shooter_.Update(input, true);

  plant.Update((Eigen::Matrix<double, 1, 1>() << 0.).finished());

  EXPECT_FALSE(output->hood_solenoid());
  EXPECT_EQ(goal->goal_mode(), c2017::shooter::ShotMode::FENDER);

  EXPECT_NEAR(plant.x(0), 0, 1e-5);
  EXPECT_NEAR(plant.x(1), 0, 1e-5);
  EXPECT_NEAR(plant.x(2), 0, 1e-5);
}

TEST(ShooterControllerTest, HopperMode) {
  c2017::shooter::ShooterInputProto input;
  c2017::shooter::ShooterOutputProto output;
  c2017::shooter::ShooterGoalProto goal;

  c2017::shooter::ShooterController shooter_;

  auto plant = muan::control::StateSpacePlant<1, 3, 1>(frc1678::shooter_controller::controller::A(),
                                                       frc1678::shooter_controller::controller::B(),
                                                       frc1678::shooter_controller::controller::C());

  plant.x(0) = 0.0;
  plant.x(1) = 0.0;
  plant.x(2) = 0.0;

  goal->set_goal_mode(c2017::shooter::ShotMode::HOPPER);

  shooter_.SetGoal(goal);

  output = shooter_.Update(input, true);

  plant.Update((Eigen::Matrix<double, 1, 1>() << 0.).finished());

  EXPECT_TRUE(output->hood_solenoid());
  EXPECT_EQ(goal->goal_mode(), c2017::shooter::ShotMode::HOPPER);

  EXPECT_NEAR(plant.x(0), 0, 1e-5);
  EXPECT_NEAR(plant.x(1), 0, 1e-5);
  EXPECT_NEAR(plant.x(2), 0, 1e-5);
}

TEST(ShooterControllerTest, SaturationTest) {
  c2017::shooter::ShooterInputProto input;
  c2017::shooter::ShooterOutputProto output;
  c2017::shooter::ShooterGoalProto goal;

  c2017::shooter::ShooterController shooter_;

  auto plant = muan::control::StateSpacePlant<1, 3, 1>(frc1678::shooter_controller::controller::A(),
                                                       frc1678::shooter_controller::controller::B(),
                                                       frc1678::shooter_controller::controller::C());

  plant.x(0) = 0.0;
  plant.x(1) = 0.0;
  plant.x(2) = 0.0;

  for (int i = 0; i <= 300; i++) {
    input->set_encoder_position(plant.x(0));

    goal->set_goal_velocity(300.0);
    goal->set_goal_mode(c2017::shooter::FENDER);

    shooter_.SetGoal(goal);

    output = shooter_.Update(input, true);

    plant.Update((Eigen::Matrix<double, 1, 1>() << output->voltage()).finished());

    EXPECT_NEAR(output->voltage(), 0., 12.);
    EXPECT_FALSE(output->hood_solenoid());
  }

  plant.x(1) = 100;  // Disturbance

  bool saturation_works;

  for (int i = 0; i <= 500; i++) {
    auto status = c2017::QueueManager::GetInstance().shooter_status_queue().ReadLastMessage();

    saturation_works = saturation_works || status.value()->profiled_goal_velocity() < 300;

    input->set_encoder_position(plant.x(0));

    output = shooter_.Update(input, true);
    plant.Update((Eigen::Matrix<double, 1, 1>() << output->voltage()).finished());
  }

  EXPECT_TRUE(saturation_works);

  auto status = c2017::QueueManager::GetInstance().shooter_status_queue().ReadLastMessage();

  if (status) {
    EXPECT_NEAR(status.value()->observed_velocity(), 300, 10);
  } else {
    FAIL();
  }
}
