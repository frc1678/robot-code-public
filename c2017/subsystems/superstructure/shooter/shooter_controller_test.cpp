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
    input->set_shooter_encoder_position(plant.x(0));

    goal->set_goal_velocity(300.0);

    shooter_.SetGoal(goal);

    output = shooter_.Update(input, true);

    plant.Update((Eigen::Matrix<double, 1, 1>() << output->shooter_voltage()).finished());

    EXPECT_NEAR(output->shooter_voltage(), 0., 12.);
    EXPECT_NEAR(output->accelerator_voltage(), 0., 12.);
  }

  auto status = c2017::QueueManager::GetInstance().shooter_status_queue().ReadLastMessage();

  if (status) {
    EXPECT_NEAR(status.value()->observed_velocity(), 300, 10);
    EXPECT_NEAR(status.value()->accelerator_observed_velocity(), 150, 20);
  } else {
    FAIL();
  }
}

TEST(ShooterControllerTest, StateMachine) {
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

  goal->set_goal_velocity(0.0);
  shooter_.SetGoal(goal);
  output = shooter_.Update(input, true);

  auto status = c2017::QueueManager::GetInstance().shooter_status_queue().ReadLastMessage();
  EXPECT_EQ(status.value()->state(), c2017::shooter::IDLE);

  goal->set_goal_velocity(300.0);
  shooter_.SetGoal(goal);
  output = shooter_.Update(input, true);

  status = c2017::QueueManager::GetInstance().shooter_status_queue().ReadLastMessage();
  EXPECT_EQ(status.value()->state(), c2017::shooter::SPINUP);

  for (int i = 0; i <= 1e4; i++) {
    input->set_shooter_encoder_position(plant.x(0));
    goal->set_goal_velocity(300.0);
    shooter_.SetGoal(goal);
    output = shooter_.Update(input, true);
    plant.Update((Eigen::Matrix<double, 1, 1>() << output->shooter_voltage()).finished());
  }

  status = c2017::QueueManager::GetInstance().shooter_status_queue().ReadLastMessage();
  EXPECT_EQ(status.value()->state(), c2017::shooter::AT_GOAL);
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
    input->set_shooter_encoder_position(plant.x(0));

    goal->set_goal_velocity(-300.0);

    shooter_.SetGoal(goal);

    output = shooter_.Update(input, true);
    status = c2017::QueueManager::GetInstance().shooter_status_queue().ReadLastMessage();

    plant.Update((Eigen::Matrix<double, 1, 1>() << output->shooter_voltage()).finished());

    EXPECT_NEAR(output->shooter_voltage(), 0., 12.);
    EXPECT_NEAR(output->accelerator_voltage(), 0., 12.);
  }

  EXPECT_EQ(output->shooter_voltage(), 0);
  EXPECT_EQ(output->accelerator_voltage(), 0);
  if (status) {
    EXPECT_NEAR(status.value()->observed_velocity(), 0, 1e-5);
    EXPECT_NEAR(status.value()->accelerator_observed_velocity(), 0, 1e-5);
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
    input->set_shooter_encoder_position(plant.x(0));

    goal->set_goal_velocity(0.0);

    shooter_.SetGoal(goal);

    output = shooter_.Update(input, true);
    status = c2017::QueueManager::GetInstance().shooter_status_queue().ReadLastMessage();

    plant.Update((Eigen::Matrix<double, 1, 1>() << output->shooter_voltage()).finished());

    EXPECT_NEAR(output->shooter_voltage(), 0., 12.);
    EXPECT_NEAR(output->accelerator_voltage(), 0., 12.);
  }

  if (status) {
    EXPECT_NEAR(status.value()->observed_velocity(), 0, 1e-5);
    EXPECT_NEAR(status.value()->accelerator_observed_velocity(), 0, 1e-5);
  } else {
    FAIL();
  }
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
    input->set_shooter_encoder_position(plant.x(0));

    goal->set_goal_velocity(300.0);

    shooter_.SetGoal(goal);

    output = shooter_.Update(input, true);

    plant.Update((Eigen::Matrix<double, 1, 1>() << output->shooter_voltage()).finished());

    EXPECT_NEAR(output->shooter_voltage(), 0., 12.);
    EXPECT_NEAR(output->accelerator_voltage(), 0., 12.);
  }

  plant.x(1) = 100;  // Disturbance

  bool saturation_works;

  for (int i = 0; i <= 500; i++) {
    auto status = c2017::QueueManager::GetInstance().shooter_status_queue().ReadLastMessage();

    saturation_works = saturation_works || status.value()->profiled_goal_velocity() < 300;

    input->set_shooter_encoder_position(plant.x(0));

    output = shooter_.Update(input, true);
    plant.Update((Eigen::Matrix<double, 1, 1>() << output->shooter_voltage()).finished());
  }

  EXPECT_TRUE(saturation_works);

  auto status = c2017::QueueManager::GetInstance().shooter_status_queue().ReadLastMessage();

  if (status) {
    EXPECT_NEAR(status.value()->observed_velocity(), 300, 10);
    EXPECT_NEAR(status.value()->accelerator_observed_velocity(), 150, 15);
  } else {
    FAIL();
  }
}
