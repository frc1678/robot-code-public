#include "drivetrain_controller.h"
#include "drivetrain_subsystem.h"
#include "o2016/subsystems/drivetrain/drivetrain_constants.h"
#include "gtest/gtest.h"

using namespace o2016::drivetrain;
using namespace frc1678::drivetrain::controller;
using namespace o2016::drivetrain::controller;

class DrivetrainTest : public testing::Test {
 public:
  DrivetrainTest() = default;
  void SetUp() override { SetGear(Gear::kLowGear); }

 protected:
  void SetGear(Gear gear) {
    if (gear == Gear::kLowGear) {
      plant_.A() = low_gear::A();
      plant_.B() = low_gear::B();
      plant_.C() = low_gear::C();
      plant_.D() = low_gear::D();
    } else {
      plant_.A() = high_gear::A();
      plant_.B() = high_gear::B();
      plant_.C() = high_gear::C();
      plant_.D() = high_gear::D();
    }

    plant_.x(0) = plant_.x(1) = plant_.x(2) = plant_.x(3) = 0.0;
  }

  DrivetrainInputProto GetSensors() const {
    DrivetrainInputProto input;
    input->set_right_encoder(plant_.y(0));
    input->set_left_encoder(plant_.y(1));
    input->set_gyro_angle(plant_.y(2));
    return input;
  }

  DrivetrainController controller_;
  muan::control::StateSpacePlant<2, 4, 3> plant_;
};

Eigen::Matrix<double, 2, 1> CreateU(DrivetrainOutputProto output) {
  return (Eigen::Matrix<double, 2, 1>() << output->right_voltage(),
          output->left_voltage())
      .finished();
}

DrivetrainGoalProto CreateVelocityGoal(double forward, double angular,
                                       Gear gear = Gear::kLowGear) {
  DrivetrainGoalProto goal;
  goal->mutable_velocity_command()->set_angular_velocity(angular);
  goal->mutable_velocity_command()->set_forward_velocity(forward);
  goal->set_gear(gear);
  return goal;
}

DrivetrainGoalProto CreateDistanceGoal(double forward_distance, double heading,
                                       Gear gear = Gear::kLowGear,
                                       double fv_final = 0.0,
                                       double av_final = 0.0) {
  DrivetrainGoalProto goal;
  auto state = goal->mutable_distance_command()->mutable_final_state();
  state->set_forward_distance(forward_distance);
  state->set_heading(heading);
  state->set_forward_velocity(fv_final);
  state->set_angular_velocity(av_final);
  goal->set_gear(gear);
  return goal;
}

TEST_F(DrivetrainTest, PlantIsSane) {
  // Forwards
  {
    using namespace muan::units;
    Velocity last_velocity = 0.0;
    Length last_position = 0.0;
    for (Time t = 0 * s; t < 1 * s; t += low_gear::dt()) {
      plant_.Update((Eigen::Matrix<double, 2, 1>() << 6, 6).finished());

      ASSERT_GE(plant_.x(0), last_position);
      ASSERT_GE(plant_.x(1), last_velocity);

      last_velocity = plant_.x(1);
      last_position = plant_.x(0);

      // It shouldn't be turning
      ASSERT_EQ(plant_.x(2), 0.0);
    }

    for (Time t = 0 * s; t < 3 * s; t += low_gear::dt()) {
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
    for (Time t = 0 * s; t < 1 * s; t += low_gear::dt()) {
      plant_.Update((Eigen::Matrix<double, 2, 1>() << 6, -6).finished());

      ASSERT_GE(plant_.x(2), last_angle);
      ASSERT_GE(plant_.x(3), last_angular_velocity);

      last_angular_velocity = plant_.x(3);
      last_angle = plant_.x(2);

      // It shouldn't be going forward
      ASSERT_EQ(plant_.x(0), 0.0);
    }

    for (Time t = 0 * s; t < 3 * s; t += low_gear::dt()) {
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
  controller_.SetGoal(CreateVelocityGoal(1.0, 1.0));
  for (Time t = 0 * s; t < 2 * s; t += low_gear::dt()) {
    auto output = controller_.Update(GetSensors());

    EXPECT_NEAR(output->left_voltage(), 0.0, 12.0);
    EXPECT_NEAR(output->right_voltage(), 0.0, 12.0);

    plant_.Update(CreateU(output));
  }

  ASSERT_NEAR(plant_.x(1), 1.0, 5e-2);
  ASSERT_NEAR(plant_.x(3), 1.0, 5e-2);
}

TEST_F(DrivetrainTest, TeleopVelocityDriveLowVoltage) {
  using namespace muan::units;
  controller_.SetGoal(CreateVelocityGoal(1.0, 1.0));
  for (Time t = 0 * s; t < 2 * s; t += low_gear::dt()) {
    auto output = controller_.Update(GetSensors());

    EXPECT_NEAR(output->left_voltage(), 0.0, 12.0);
    EXPECT_NEAR(output->right_voltage(), 0.0, 12.0);

    // Pretend one of the drivetrain motors is dead and the world is a sad place
    output->set_left_voltage(output->left_voltage() * 1.0);
    output->set_right_voltage(output->right_voltage() * 0.7);

    plant_.Update(CreateU(output));
  }

  ASSERT_NEAR(plant_.x(1), 1.0, 5e-2);
  ASSERT_NEAR(plant_.x(3), 1.0, 5e-2);
}

TEST_F(DrivetrainTest, TeleopShiftDuringDrive) {
  using namespace muan::units;
  controller_.SetGoal(CreateVelocityGoal(1.0, 1.0));
  for (Time t = 0 * s; t < 2 * s; t += low_gear::dt()) {
    auto output = controller_.Update(GetSensors());

    EXPECT_NEAR(output->left_voltage(), 0.0, 12.0);
    EXPECT_NEAR(output->right_voltage(), 0.0, 12.0);
    EXPECT_EQ(output->high_gear(), false);

    plant_.Update(CreateU(output));
  }

  // Run in high gear
  SetGear(Gear::kHighGear);

  controller_.SetGoal(CreateVelocityGoal(2.0, 2.0, Gear::kHighGear));
  for (Time t = 0 * s; t < 4 * s; t += low_gear::dt()) {
    auto output = controller_.Update(GetSensors());

    EXPECT_NEAR(output->left_voltage(), 0.0, 12.0);
    EXPECT_NEAR(output->right_voltage(), 0.0, 12.0);
    EXPECT_EQ(output->high_gear(), true);

    plant_.Update(CreateU(output));
  }
  ASSERT_NEAR(plant_.x(1), 2.0, 5e-2);
  ASSERT_NEAR(plant_.x(3), 2.0, 5e-2);
}

TEST_F(DrivetrainTest, AutoDriveDistance) {
  using namespace muan::units;
  controller_.SetGoal(CreateDistanceGoal(1.0, 0.0));

  bool finished = false;
  for (Time t = 0 * s; t < 20 * s; t += low_gear::dt()) {
    auto output = controller_.Update(GetSensors());

    EXPECT_NEAR(output->left_voltage(), 0.0, 12.0);
    EXPECT_NEAR(output->right_voltage(), 0.0, 12.0);
    EXPECT_EQ(output->high_gear(), false);

    plant_.Update(CreateU(output));

    auto status = controller_.GetStatus();
    finished = finished || status->just_finished_profile();
  }

  ASSERT_TRUE(finished);
  ASSERT_NEAR(plant_.x(0), 1.0, 5e-2);
  ASSERT_NEAR(plant_.x(1), 0.0, 5e-2);
}

TEST_F(DrivetrainTest, AutoTurn) {
  using namespace muan::units;
  controller_.SetGoal(CreateDistanceGoal(0.0, 1.0));

  bool finished = false;
  for (Time t = 0 * s; t < 2 * s; t += low_gear::dt()) {
    auto output = controller_.Update(GetSensors());

    EXPECT_NEAR(output->left_voltage(), 0.0, 12.0);
    EXPECT_NEAR(output->right_voltage(), 0.0, 12.0);
    EXPECT_EQ(output->high_gear(), false);

    plant_.Update(CreateU(output));

    auto status = controller_.GetStatus();
    finished = finished || status->just_finished_profile();
  }

  ASSERT_TRUE(finished);
  ASSERT_NEAR(plant_.x(2), 1.0, 5e-2);
  ASSERT_NEAR(plant_.x(3), 0.0, 5e-2);
}

TEST(DrivetrainSubsystem, GoalInterrupts) {
  // Write a single input at the beginning. Since we're not clearing the input
  // queue, this will just remain forever as the last message. It's a sketchy
  // hack but it works.
  o2016::QueueManager::GetInstance().drivetrain_input_queue().WriteMessage(
      DrivetrainInputProto());

  {
    // Velocity message should not interrupt distance messages
    DrivetrainSubsystem subsystem;

    o2016::QueueManager::GetInstance().drivetrain_goal_queue().Reset();

    o2016::QueueManager::GetInstance().drivetrain_goal_queue().WriteMessage(
        CreateDistanceGoal(1.0, 1.0));
    subsystem.Update();

    o2016::QueueManager::GetInstance().drivetrain_goal_queue().WriteMessage(
        CreateVelocityGoal(1.0, 1.0));
    subsystem.Update();

    auto status = o2016::QueueManager::GetInstance()
                      .drivetrain_status_queue()
                      .MakeReader()
                      .ReadLastMessage()
                      .value();
    EXPECT_EQ(status->current_driving_type(), DriveType::kDistanceCommand);
  }

  {
    // Distance message should interrupt velocity message
    DrivetrainSubsystem subsystem;

    o2016::QueueManager::GetInstance().drivetrain_goal_queue().Reset();

    o2016::QueueManager::GetInstance().drivetrain_goal_queue().WriteMessage(
        CreateVelocityGoal(1.0, 0.0));
    subsystem.Update();

    // Make sure it did, in fact, start using the velocity goal
    auto status = o2016::QueueManager::GetInstance()
                      .drivetrain_status_queue()
                      .MakeReader()
                      .ReadLastMessage()
                      .value();
    EXPECT_EQ(status->current_driving_type(), DriveType::kVelocityCommand);

    o2016::QueueManager::GetInstance().drivetrain_goal_queue().WriteMessage(
        CreateDistanceGoal(1.0, 1.0));
    subsystem.Update();

    status = o2016::QueueManager::GetInstance()
                 .drivetrain_status_queue()
                 .MakeReader()
                 .ReadLastMessage()
                 .value();
    EXPECT_EQ(status->current_driving_type(), DriveType::kDistanceCommand);
  }
  {
    // New distance message should interrupt old distance message
    DrivetrainSubsystem subsystem;

    o2016::QueueManager::GetInstance().drivetrain_goal_queue().Reset();

    o2016::QueueManager::GetInstance().drivetrain_goal_queue().WriteMessage(
        CreateDistanceGoal(1.0, 0.0));
    subsystem.Update();

    auto status = o2016::QueueManager::GetInstance()
                      .drivetrain_status_queue()
                      .MakeReader()
                      .ReadLastMessage()
                      .value();
    EXPECT_EQ(status->num_profiles_run(), 0);

    o2016::QueueManager::GetInstance().drivetrain_goal_queue().WriteMessage(
        CreateDistanceGoal(0.0, 1.0));
    subsystem.Update();

    status = o2016::QueueManager::GetInstance()
                 .drivetrain_status_queue()
                 .MakeReader()
                 .ReadLastMessage()
                 .value();
    EXPECT_EQ(status->current_driving_type(), DriveType::kDistanceCommand);
    EXPECT_EQ(status->num_profiles_run(), 1);
    EXPECT_EQ(status->profile_goal().heading(), 1.0);
  }
}

/*
 * TODO(Kyle): Tests I need to write
 *  Auto
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
