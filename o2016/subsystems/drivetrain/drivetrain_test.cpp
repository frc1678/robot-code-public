#include "drivetrain_controller.h"
#include "o2016/subsystems/drivetrain/drivetrain_constants.h"
#include "gtest/gtest.h"

TEST(DrivetrainController, PlantIsSane) {
  frc1678::drivetrain::controller::DrivetrainController controller;
  muan::control::StateSpacePlant<2, 4, 3> plant;

  plant.A() = frc1678::drivetrain::controller::low_gear::A();
  plant.B() = frc1678::drivetrain::controller::low_gear::B();
  plant.C() = frc1678::drivetrain::controller::low_gear::C();
  plant.D() = frc1678::drivetrain::controller::low_gear::D();

  // Forwards
  {
    using namespace muan::units;
    Velocity last_velocity = 0.0;
    Length last_position = 0.0;
    for (Time t = 0 * s; t < 1 * s;
         t += frc1678::drivetrain::controller::low_gear::dt()) {
      plant.Update((Eigen::Matrix<double, 2, 1>() << 6, 6).finished());

      ASSERT_GE(plant.x(0), last_position);
      ASSERT_GE(plant.x(1), last_velocity);

      last_velocity = plant.x(1);
      last_position = plant.x(0);

      // It shouldn't be turning
      ASSERT_EQ(plant.x(2), 0.0);
    }

    for (Time t = 0 * s; t < 3 * s;
         t += frc1678::drivetrain::controller::low_gear::dt()) {
      plant.Update(Eigen::Matrix<double, 2, 1>::Zero());

      ASSERT_GE(plant.x(0), last_position);
      ASSERT_LE(plant.x(1), last_velocity);

      last_velocity = plant.x(1);
      last_position = plant.x(0);
    }
    ASSERT_NEAR(plant.x(1), 0.0, 1e-2);
  }

  // Reset the drivetrain
  plant.x(0) = plant.x(1) = plant.x(2) = plant.x(3) = 0.0;

  // Turning
  {
    using namespace muan::units;
    AngularVelocity last_angular_velocity = 0.0;
    Angle last_angle = 0.0;
    for (Time t = 0 * s; t < 1 * s;
         t += frc1678::drivetrain::controller::low_gear::dt()) {
      plant.Update((Eigen::Matrix<double, 2, 1>() << 6, -6).finished());

      ASSERT_GE(plant.x(2), last_angle);
      ASSERT_GE(plant.x(3), last_angular_velocity);

      last_angular_velocity = plant.x(3);
      last_angle = plant.x(2);

      // It shouldn't be going forward
      ASSERT_EQ(plant.x(0), 0.0);
    }

    for (Time t = 0 * s; t < 3 * s;
         t += frc1678::drivetrain::controller::low_gear::dt()) {
      plant.Update(Eigen::Matrix<double, 2, 1>::Zero());

      ASSERT_GE(plant.x(2), last_angle);
      ASSERT_LE(plant.x(3), last_angular_velocity);

      last_angular_velocity = plant.x(3);
      last_angle = plant.x(1);
    }
    ASSERT_NEAR(plant.x(1), 0.0, 1e-2);
  }
}

TEST(DrivetrainController, TeleopVelocityDrive) {
  frc1678::drivetrain::controller::DrivetrainController controller;
  muan::control::StateSpacePlant<2, 4, 3> plant;

  plant.A() = frc1678::drivetrain::controller::low_gear::A();
  plant.B() = frc1678::drivetrain::controller::low_gear::B();
  plant.C() = frc1678::drivetrain::controller::low_gear::C();
  plant.D() = frc1678::drivetrain::controller::low_gear::D();

  plant.x(0) = plant.x(1) = 0.0;
  using namespace muan::units;
  for (Time t = 0 * s; t < 2 * s;
       t += frc1678::drivetrain::controller::low_gear::dt()) {
    DrivetrainInput input;
    input.set_right_encoder(plant.y(0));
    input.set_left_encoder(plant.y(1));
    input.set_gyro_angle(plant.y(2));

    DrivetrainGoal goal;
    goal.mutable_velocity_command()->set_forward_velocity(1.0);
    goal.mutable_velocity_command()->set_angular_velocity(1.0);
    goal.set_gear(Gear::kLowGear);

    auto u = controller.Update(input, goal);
    plant.Update(
        (Eigen::Matrix<double, 2, 1>() << u.right_voltage(), u.left_voltage())
            .finished());
  }

  ASSERT_NEAR(plant.x(1), 1.0, 5e-2);
  ASSERT_NEAR(plant.x(3), 1.0, 5e-2);
}

TEST(DrivetrainController, TeleopVelocityDriveLowVoltage) {
  frc1678::drivetrain::controller::DrivetrainController controller;
  muan::control::StateSpacePlant<2, 4, 3> plant;

  plant.A() = frc1678::drivetrain::controller::low_gear::A();
  plant.B() = frc1678::drivetrain::controller::low_gear::B();
  plant.C() = frc1678::drivetrain::controller::low_gear::C();
  plant.D() = frc1678::drivetrain::controller::low_gear::D();

  plant.x(0) = plant.x(1) = 0.0;
  using namespace muan::units;
  for (Time t = 0 * s; t < 2 * s;
       t += frc1678::drivetrain::controller::low_gear::dt()) {
    DrivetrainInput input;
    input.set_right_encoder(plant.y(0));
    input.set_left_encoder(plant.y(1));
    input.set_gyro_angle(plant.y(2));

    DrivetrainGoal goal;
    goal.mutable_velocity_command()->set_forward_velocity(1.0);
    goal.mutable_velocity_command()->set_angular_velocity(1.0);
    goal.set_gear(Gear::kLowGear);

    auto u = controller.Update(input, goal);

    // Pretend one of the drivetrain motors is dead and the world is a sad place
    plant.Update((Eigen::Matrix<double, 2, 1>() << u.right_voltage() * .7,
                  u.left_voltage() * 1.1)
                     .finished());
  }

  ASSERT_NEAR(plant.x(1), 1.0, 5e-2);
  ASSERT_NEAR(plant.x(3), 1.0, 5e-2);
}

TEST(DrivetrainController, TeleopShiftDuringDrive) {
  frc1678::drivetrain::controller::DrivetrainController controller;
  muan::control::StateSpacePlant<2, 4, 3> plant;

  plant.A() = frc1678::drivetrain::controller::low_gear::A();
  plant.B() = frc1678::drivetrain::controller::low_gear::B();
  plant.C() = frc1678::drivetrain::controller::low_gear::C();
  plant.D() = frc1678::drivetrain::controller::low_gear::D();

  plant.x(0) = plant.x(1) = 0.0;
  using namespace muan::units;
  for (Time t = 0 * s; t < 2 * s;
       t += frc1678::drivetrain::controller::low_gear::dt()) {
    DrivetrainInput input;
    input.set_right_encoder(plant.y(0));
    input.set_left_encoder(plant.y(1));
    input.set_gyro_angle(plant.y(2));

    DrivetrainGoal goal;
    goal.mutable_velocity_command()->set_forward_velocity(1.0);
    goal.mutable_velocity_command()->set_angular_velocity(1.0);
    goal.set_gear(Gear::kLowGear);

    auto u = controller.Update(input, goal);

    // Pretend one of the drivetrain motors is dead and the world is a sad place
    plant.Update((Eigen::Matrix<double, 2, 1>() << u.right_voltage() * .7,
                  u.left_voltage() * 1.1)
                     .finished());
  }

  // Run in high gear
  plant.A() = frc1678::drivetrain::controller::high_gear::A();
  plant.B() = frc1678::drivetrain::controller::high_gear::B();
  plant.C() = frc1678::drivetrain::controller::high_gear::C();
  plant.D() = frc1678::drivetrain::controller::high_gear::D();

  for (Time t = 0 * s; t < 2 * s;
       t += frc1678::drivetrain::controller::low_gear::dt()) {
    DrivetrainInput input;
    input.set_right_encoder(plant.y(0));
    input.set_left_encoder(plant.y(1));
    input.set_gyro_angle(plant.y(2));

    DrivetrainGoal goal;
    goal.mutable_velocity_command()->set_forward_velocity(2.0);
    goal.mutable_velocity_command()->set_angular_velocity(2.0);
    goal.set_gear(Gear::kHighGear);

    auto u = controller.Update(input, goal);

    // Pretend one of the drivetrain motors is dead and the world is a sad place
    plant.Update(
        (Eigen::Matrix<double, 2, 1>() << u.right_voltage(), u.left_voltage())
            .finished());
  }
  ASSERT_NEAR(plant.x(1), 2.0, 5e-2);
  ASSERT_NEAR(plant.x(3), 2.0, 5e-2);
}

/*
 * TODO(Kyle): Tests I need to write
 *  Auto
 *   - Drives distance
 *   - Distance with initial velocity
 *   - Carries out motion profile through disturbance
 *   - Obeys any extra constraints from the goal
 *  Disabled
 *   - Survives disabling
 *   - Correctly observes while disabled
 *   - Does not output voltage while disabled
 *   - Doesn't try to shift while disabled, and keeps correct internal knowledge
 *      of gear
 */
