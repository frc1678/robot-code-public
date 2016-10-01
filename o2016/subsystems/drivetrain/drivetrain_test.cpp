#include "drivetrain_controller.h"
#include "o2016/subsystems/drivetrain/drivetrain_constants.h"
#include "gtest/gtest.h"

class DrivetrainTest : public testing::Test {
 public:
  DrivetrainTest() = default;
  void SetUp() override { SetGear(Gear::kLowGear); }

 protected:
  void SetGear(Gear gear) {
    if (gear == Gear::kLowGear) {
      plant_.A() = frc1678::drivetrain::controller::low_gear::A();
      plant_.B() = frc1678::drivetrain::controller::low_gear::B();
      plant_.C() = frc1678::drivetrain::controller::low_gear::C();
      plant_.D() = frc1678::drivetrain::controller::low_gear::D();
    } else {
      plant_.A() = frc1678::drivetrain::controller::high_gear::A();
      plant_.B() = frc1678::drivetrain::controller::high_gear::B();
      plant_.C() = frc1678::drivetrain::controller::high_gear::C();
      plant_.D() = frc1678::drivetrain::controller::high_gear::D();
    }

    plant_.x(0) = plant_.x(1) = plant_.x(2) = plant_.x(3) = 0.0;
  }

  DrivetrainInput GetSensors() const {
    DrivetrainInput input;
    input.set_right_encoder(plant_.y(0));
    input.set_left_encoder(plant_.y(1));
    input.set_gyro_angle(plant_.y(2));
    return input;
  }

  frc1678::drivetrain::controller::DrivetrainController controller_;
  muan::control::StateSpacePlant<2, 4, 3> plant_;
};

Eigen::Matrix<double, 2, 1> CreateU(DrivetrainOutput output) {
  return (Eigen::Matrix<double, 2, 1>() << output.right_voltage(),
          output.left_voltage())
      .finished();
}

DrivetrainGoal CreateVelocityGoal(double forward, double angular,
                                  Gear gear = Gear::kLowGear) {
  DrivetrainGoal goal;
  goal.mutable_velocity_command()->set_angular_velocity(angular);
  goal.mutable_velocity_command()->set_forward_velocity(forward);
  goal.set_gear(gear);
  return goal;
}

TEST_F(DrivetrainTest, PlantIsSane) {
  // Forwards
  {
    using namespace muan::units;
    Velocity last_velocity = 0.0;
    Length last_position = 0.0;
    for (Time t = 0 * s; t < 1 * s;
         t += frc1678::drivetrain::controller::low_gear::dt()) {
      plant_.Update((Eigen::Matrix<double, 2, 1>() << 6, 6).finished());

      ASSERT_GE(plant_.x(0), last_position);
      ASSERT_GE(plant_.x(1), last_velocity);

      last_velocity = plant_.x(1);
      last_position = plant_.x(0);

      // It shouldn't be turning
      ASSERT_EQ(plant_.x(2), 0.0);
    }

    for (Time t = 0 * s; t < 3 * s;
         t += frc1678::drivetrain::controller::low_gear::dt()) {
      plant_.Update(Eigen::Matrix<double, 2, 1>::Zero());

      ASSERT_GE(plant_.x(0), last_position);
      ASSERT_LE(plant_.x(1), last_velocity);

      last_velocity = plant_.x(1);
      last_position = plant_.x(0);
    }
    ASSERT_NEAR(plant_.x(1), 0.0, 1e-2);
  }

  // Reset the drivetrain
  SetUp();

  // Turning
  {
    using namespace muan::units;
    AngularVelocity last_angular_velocity = 0.0;
    Angle last_angle = 0.0;
    for (Time t = 0 * s; t < 1 * s;
         t += frc1678::drivetrain::controller::low_gear::dt()) {
      plant_.Update((Eigen::Matrix<double, 2, 1>() << 6, -6).finished());

      ASSERT_GE(plant_.x(2), last_angle);
      ASSERT_GE(plant_.x(3), last_angular_velocity);

      last_angular_velocity = plant_.x(3);
      last_angle = plant_.x(2);

      // It shouldn't be going forward
      ASSERT_EQ(plant_.x(0), 0.0);
    }

    for (Time t = 0 * s; t < 3 * s;
         t += frc1678::drivetrain::controller::low_gear::dt()) {
      plant_.Update(Eigen::Matrix<double, 2, 1>::Zero());

      ASSERT_GE(plant_.x(2), last_angle);
      ASSERT_LE(plant_.x(3), last_angular_velocity);

      last_angular_velocity = plant_.x(3);
      last_angle = plant_.x(1);
    }
    ASSERT_NEAR(plant_.x(1), 0.0, 1e-2);
  }
}

TEST_F(DrivetrainTest, TeleopVelocityDrive) {
  using namespace muan::units;
  for (Time t = 0 * s; t < 2 * s;
       t += frc1678::drivetrain::controller::low_gear::dt()) {
    auto output =
        controller_.Update(GetSensors(), CreateVelocityGoal(1.0, 1.0));

    EXPECT_NEAR(output.left_voltage(), 0.0, 12.0);
    EXPECT_NEAR(output.right_voltage(), 0.0, 12.0);

    plant_.Update(CreateU(output));
  }

  ASSERT_NEAR(plant_.x(1), 1.0, 5e-2);
  ASSERT_NEAR(plant_.x(3), 1.0, 5e-2);
}

TEST_F(DrivetrainTest, TeleopVelocityDriveLowVoltage) {
  using namespace muan::units;
  for (Time t = 0 * s; t < 2 * s;
       t += frc1678::drivetrain::controller::low_gear::dt()) {
    auto output =
        controller_.Update(GetSensors(), CreateVelocityGoal(1.0, 1.0));

    EXPECT_NEAR(output.left_voltage(), 0.0, 12.0);
    EXPECT_NEAR(output.right_voltage(), 0.0, 12.0);

    // Pretend one of the drivetrain motors is dead and the world is a sad place
    output.set_left_voltage(output.left_voltage() * 1.0);
    output.set_right_voltage(output.right_voltage() * 0.7);

    plant_.Update(CreateU(output));
  }

  ASSERT_NEAR(plant_.x(1), 1.0, 5e-2);
  ASSERT_NEAR(plant_.x(3), 1.0, 5e-2);
}

TEST_F(DrivetrainTest, TeleopShiftDuringDrive) {
  using namespace muan::units;
  for (Time t = 0 * s; t < 2 * s;
       t += frc1678::drivetrain::controller::low_gear::dt()) {
    auto output =
        controller_.Update(GetSensors(), CreateVelocityGoal(1.0, 1.0));

    EXPECT_NEAR(output.left_voltage(), 0.0, 12.0);
    EXPECT_NEAR(output.right_voltage(), 0.0, 12.0);
    EXPECT_EQ(output.shifting(), false);

    plant_.Update(CreateU(output));
  }

  // Run in high gear
  SetGear(Gear::kHighGear);

  for (Time t = 0 * s; t < 4 * s;
       t += frc1678::drivetrain::controller::low_gear::dt()) {
    auto output = controller_.Update(
        GetSensors(), CreateVelocityGoal(2.0, 2.0, Gear::kHighGear));

    EXPECT_NEAR(output.left_voltage(), 0.0, 12.0);
    EXPECT_NEAR(output.right_voltage(), 0.0, 12.0);
    EXPECT_EQ(output.shifting(), true);

    plant_.Update(CreateU(output));
  }
  ASSERT_NEAR(plant_.x(1), 2.0, 5e-2);
  ASSERT_NEAR(plant_.x(3), 2.0, 5e-2);
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
