#include "c2017/subsystems/superstructure/shooter/shooter_controller.h"
#include "c2017/subsystems/superstructure/shooter/queue_types.h"
#include "c2017/subsystems/superstructure/shooter/shooter.pb.h"
#include "gtest/gtest.h"
#include "muan/wpilib/queue_types.h"

namespace c2017 {
namespace shooter {

TEST(ShooterControllerTest, IsSane) {
  auto plant = muan::control::StateSpacePlant<1, 3, 1>(
      frc1678::shooter_controller::controller::A(),
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

  auto plant = muan::control::StateSpacePlant<1, 3, 1>(
      frc1678::shooter_controller::controller::A(),
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

    plant.Update((Eigen::Matrix<double, 1, 1>() << output->shooter_voltage())
                     .finished());

    EXPECT_NEAR(output->shooter_voltage(), 0., 12.);
    EXPECT_NEAR(output->accelerator_voltage(), 0., 12.);
  }

  auto status = c2017::QueueManager::GetInstance()
                    ->shooter_status_queue()
                    ->ReadLastMessage();

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

  auto plant = muan::control::StateSpacePlant<1, 3, 1>(
      frc1678::shooter_controller::controller::A(),
      frc1678::shooter_controller::controller::B(),
      frc1678::shooter_controller::controller::C());

  plant.x(0) = 0.0;
  plant.x(1) = 0.0;
  plant.x(2) = 0.0;

  output = shooter_.Update(input, true);
  goal->set_goal_velocity(0.0);
  shooter_.SetGoal(goal);
  output = shooter_.Update(input, true);

  auto status = c2017::QueueManager::GetInstance()
                    ->shooter_status_queue()
                    ->ReadLastMessage();
  EXPECT_EQ(status.value()->state(), c2017::shooter::IDLE);

  goal->set_goal_velocity(300.0);
  shooter_.SetGoal(goal);
  output = shooter_.Update(input, true);

  status = c2017::QueueManager::GetInstance()
               ->shooter_status_queue()
               ->ReadLastMessage();
  EXPECT_EQ(status.value()->state(), c2017::shooter::SPINUP);

  for (int i = 0; i <= 1e4; i++) {
    input->set_shooter_encoder_position(plant.x(0));
    goal->set_goal_velocity(300.0);
    shooter_.SetGoal(goal);
    output = shooter_.Update(input, true);
    plant.Update((Eigen::Matrix<double, 1, 1>() << output->shooter_voltage())
                     .finished());
  }

  status = c2017::QueueManager::GetInstance()
               ->shooter_status_queue()
               ->ReadLastMessage();
  EXPECT_EQ(status.value()->state(), c2017::shooter::AT_GOAL);
}

TEST(ShooterControllerTest, CantTakeNegativeVoltage) {
  c2017::shooter::ShooterInputProto input;
  c2017::shooter::ShooterOutputProto output;
  c2017::shooter::ShooterGoalProto goal;

  auto status = c2017::QueueManager::GetInstance()
                    ->shooter_status_queue()
                    ->ReadLastMessage();

  c2017::shooter::ShooterController shooter_;

  auto plant = muan::control::StateSpacePlant<1, 3, 1>(
      frc1678::shooter_controller::controller::A(),
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
    status = c2017::QueueManager::GetInstance()
                 ->shooter_status_queue()
                 ->ReadLastMessage();

    plant.Update((Eigen::Matrix<double, 1, 1>() << output->shooter_voltage())
                     .finished());

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

  auto plant = muan::control::StateSpacePlant<1, 3, 1>(
      frc1678::shooter_controller::controller::A(),
      frc1678::shooter_controller::controller::B(),
      frc1678::shooter_controller::controller::C());

  plant.x(0) = 0.0;
  plant.x(1) = 0.0;
  plant.x(2) = 0.0;

  auto status = c2017::QueueManager::GetInstance()
                    ->shooter_status_queue()
                    ->ReadLastMessage();

  for (int i = 0; i <= 1e3; i++) {
    input->set_shooter_encoder_position(plant.x(0));

    goal->set_goal_velocity(0.0);

    shooter_.SetGoal(goal);

    output = shooter_.Update(input, true);
    status = c2017::QueueManager::GetInstance()
                 ->shooter_status_queue()
                 ->ReadLastMessage();

    plant.Update((Eigen::Matrix<double, 1, 1>() << output->shooter_voltage())
                     .finished());

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

  auto plant = muan::control::StateSpacePlant<1, 3, 1>(
      frc1678::shooter_controller::controller::A(),
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

    plant.Update((Eigen::Matrix<double, 1, 1>() << output->shooter_voltage())
                     .finished());

    EXPECT_NEAR(output->shooter_voltage(), 0., 12.);
    EXPECT_NEAR(output->accelerator_voltage(), 0., 12.);
  }

  plant.x(1) = 100;  // Disturbance

  bool saturation_works = false;

  for (int i = 0; i <= 500; i++) {
    auto status = c2017::QueueManager::GetInstance()
                      ->shooter_status_queue()
                      ->ReadLastMessage();

    saturation_works =
        saturation_works || (status.value()->profiled_goal_velocity() < 300);

    input->set_shooter_encoder_position(plant.x(0));

    output = shooter_.Update(input, true);
    plant.Update((Eigen::Matrix<double, 1, 1>() << output->shooter_voltage())
                     .finished());
  }

  EXPECT_TRUE(saturation_works);

  auto status = c2017::QueueManager::GetInstance()
                    ->shooter_status_queue()
                    ->ReadLastMessage();

  if (status) {
    EXPECT_NEAR(status.value()->observed_velocity(), 300, 10);
    EXPECT_NEAR(status.value()->accelerator_observed_velocity(), 150, 15);
  } else {
    FAIL();
  }
}

TEST(ShooterControllerTest, EncoderNeverPluggedIn) {
  c2017::shooter::ShooterInputProto input;
  c2017::shooter::ShooterOutputProto output;
  c2017::shooter::ShooterGoalProto goal;

  c2017::shooter::ShooterController shooter_;
  goal->set_goal_velocity(300);

  shooter_.SetGoal(goal);

  for (int i = 0; i <= c2017::shooter::kEncoderFaultTicksAllowed - 1; i++) {
    // Encoder unplugged, but it shouldn't be detected
    input->set_shooter_encoder_position(0);

    output = shooter_.Update(input, true);
  }

  auto status = c2017::QueueManager::GetInstance()
                    ->shooter_status_queue()
                    ->ReadLastMessage();
  ASSERT_TRUE(status);
  EXPECT_FALSE(status.value()->encoder_fault_detected());
  EXPECT_EQ(status.value()->state(), c2017::shooter::State::SPINUP);

  for (int i = 0; i <= 100; i++) {
    // Encoder fault should be detected
    input->set_shooter_encoder_position(0);

    output = shooter_.Update(input, true);
  }

  status = c2017::QueueManager::GetInstance()
               ->shooter_status_queue()
               ->ReadLastMessage();

  ASSERT_TRUE(status);
  EXPECT_TRUE(status.value()->encoder_fault_detected());
  EXPECT_NEAR(output->shooter_voltage(), c2017::shooter::kShooterOpenLoopU,
              1e-3);
  EXPECT_EQ(status.value()->state(), c2017::shooter::State::AT_GOAL);
}

TEST(ShooterControllerTest, EncoderComesUnplugged) {
  c2017::shooter::ShooterInputProto input;
  c2017::shooter::ShooterOutputProto output;
  c2017::shooter::ShooterGoalProto goal;

  c2017::shooter::ShooterController shooter_;
  goal->set_goal_velocity(300);

  shooter_.SetGoal(goal);

  for (int i = 0; i <= 300; i++) {
    // Running normally
    input->set_shooter_encoder_position(i);

    output = shooter_.Update(input, true);
  }

  auto status = c2017::QueueManager::GetInstance()
                    ->shooter_status_queue()
                    ->ReadLastMessage();
  ASSERT_TRUE(status);
  EXPECT_FALSE(status.value()->encoder_fault_detected());
  EXPECT_EQ(status.value()->state(), c2017::shooter::State::SPINUP);

  for (int i = 0; i <= c2017::shooter::kEncoderFaultTicksAllowed - 1; i++) {
    // Encoder unplugged almost until allowed ticks
    input->set_shooter_encoder_position(300);

    output = shooter_.Update(input, true);
  }

  status = c2017::QueueManager::GetInstance()
               ->shooter_status_queue()
               ->ReadLastMessage();
  ASSERT_TRUE(status);
  EXPECT_FALSE(status.value()->encoder_fault_detected());
  EXPECT_EQ(status.value()->state(), c2017::shooter::State::SPINUP);

  for (int i = 0; i <= 500; i++) {
    // Encoder fault should be detected
    input->set_shooter_encoder_position(0);

    output = shooter_.Update(input, true);
  }

  status = c2017::QueueManager::GetInstance()
               ->shooter_status_queue()
               ->ReadLastMessage();

  ASSERT_TRUE(status);
  EXPECT_TRUE(status.value()->encoder_fault_detected());
  EXPECT_NEAR(output->shooter_voltage(), c2017::shooter::kShooterOpenLoopU,
              1e-3);
  EXPECT_EQ(status.value()->state(), c2017::shooter::State::AT_GOAL);
}

TEST(ShooterControllerTest, Brownout) {
  c2017::shooter::ShooterInputProto input;
  c2017::shooter::ShooterOutputProto output;
  c2017::shooter::ShooterGoalProto goal;

  c2017::shooter::ShooterController shooter_;

  auto plant = muan::control::StateSpacePlant<1, 3, 1>(
      frc1678::shooter_controller::controller::A(),
      frc1678::shooter_controller::controller::B(),
      frc1678::shooter_controller::controller::C());

  plant.x(0) = 0.0;
  plant.x(1) = 0.0;
  plant.x(2) = 0.0;

  for (int i = 0; i <= 500; i++) {
    input->set_shooter_encoder_position(plant.x(0));

    goal->set_goal_velocity(300.0);

    shooter_.SetGoal(goal);

    output = shooter_.Update(input, true);

    plant.Update((Eigen::Matrix<double, 1, 1>() << output->shooter_voltage())
                     .finished());
  }

  EXPECT_EQ(shooter_.profiled_goal_velocity_, shooter_.profiled_goal_velocity_);

  for (int i = 0; i <= 50; i++) {
    input->set_shooter_encoder_position(plant.x(0));

    goal->set_goal_velocity(300.0);

    shooter_.SetGoal(goal);

    output = shooter_.Update(input, false);

    EXPECT_EQ(output->shooter_voltage(), 0.0);

    plant.Update((Eigen::Matrix<double, 1, 1>() << 0.0).finished());
  }

  EXPECT_NE(shooter_.profiled_goal_velocity_, 0);

  for (int i = 0; i <= 450; i++) {
    input->set_shooter_encoder_position(plant.x(0));

    goal->set_goal_velocity(300.0);

    shooter_.SetGoal(goal);

    output = shooter_.Update(input, false);

    plant.Update((Eigen::Matrix<double, 1, 1>() << output->shooter_voltage())
                     .finished());
  }
  auto status = c2017::QueueManager::GetInstance()
                    ->shooter_status_queue()
                    ->ReadLastMessage();
  EXPECT_EQ(shooter_.profiled_goal_velocity_, 0);
}

}  // namespace shooter
}  // namespace c2017
