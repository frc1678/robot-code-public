#include "gtest/gtest.h"
#include "shooter_controller.h"
#include "queue_types.h"
#include "muan/wpilib/queue_types.h"
#include "c2017/subsystems/superstructure/shooter/shooter.pb.h"

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
  muan::wpilib::DriverStationProto ds;

  auto status = c2017::QueueManager::GetInstance().shooter_status_queue().ReadLastMessage();

  ds->set_mode(RobotMode::TELEOP);

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

    output = shooter_.Update(input, ds); 
    status = c2017::QueueManager::GetInstance().shooter_status_queue().ReadLastMessage();

    plant.Update((Eigen::Matrix<double, 1, 1>() << output->voltage()).finished());

    EXPECT_NEAR(output->voltage(), 0., 12.);
  }
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
  muan::wpilib::DriverStationProto ds;

  auto status = c2017::QueueManager::GetInstance().shooter_status_queue().ReadLastMessage();

  ds->set_mode(RobotMode::TELEOP);

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

    output = shooter_.Update(input, ds);
    status = c2017::QueueManager::GetInstance().shooter_status_queue().ReadLastMessage();

    plant.Update((Eigen::Matrix<double, 1, 1>() << output->voltage()).finished());

    EXPECT_NEAR(output->voltage(), 0., 12.);
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
  muan::wpilib::DriverStationProto ds;

  ds->set_mode(RobotMode::TELEOP);

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

    output = shooter_.Update(input, ds);
    status = c2017::QueueManager::GetInstance().shooter_status_queue().ReadLastMessage();
    
    plant.Update((Eigen::Matrix<double, 1, 1>() << output->voltage()).finished());

    EXPECT_NEAR(output->voltage(), 0., 12.);
  }

  if (status) {
    EXPECT_NEAR(status.value()->observed_velocity(), 0, 1e-5);
  } else {
    FAIL();
  }
}
