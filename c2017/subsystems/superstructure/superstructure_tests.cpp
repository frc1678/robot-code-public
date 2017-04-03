#include "c2017/subsystems/superstructure/superstructure.h"

#include "c2017/queue_manager/queue_manager.h"
#include "gtest/gtest.h"

namespace c2017 {

class SuperstructureTest : public ::testing::Test {
 public:
  SuperstructureTest() {}

  void WriteQueues() {
    QueueManager::GetInstance().intake_group_goal_queue().WriteMessage(intake_group_goal_proto_);
    QueueManager::GetInstance().shooter_group_goal_queue().WriteMessage(shooter_group_goal_proto_);
    QueueManager::GetInstance().driver_station_queue()->WriteMessage(driver_station_proto_);
    QueueManager::GetInstance().climber_input_queue().WriteMessage(climber_input_proto_);
    QueueManager::GetInstance().shooter_input_queue().WriteMessage(shooter_input_proto_);
    QueueManager::GetInstance().ground_gear_input_queue().WriteMessage(ground_gear_input_proto_);
  }

  void SetUp() override {
    intake_group_goal_proto_.Reset();
    shooter_group_goal_proto_.Reset();
    driver_station_proto_.Reset();
    shooter_status_proto_.Reset();
    climber_input_proto_.Reset();
    shooter_input_proto_.Reset();
    ground_gear_input_proto_.Reset();

    QueueManager::GetInstance().Reset();
  }

 protected:
  intake_group::IntakeGroupGoalProto intake_group_goal_proto_;
  shooter_group::ShooterGroupGoalProto shooter_group_goal_proto_;
  muan::wpilib::DriverStationProto driver_station_proto_;
  superstructure::SuperStructure superstructure;
  shooter::ShooterStatusProto shooter_status_proto_;

  climber::ClimberInputProto climber_input_proto_;
  shooter::ShooterInputProto shooter_input_proto_;
  ground_gear_intake::GroundGearIntakeInputProto ground_gear_input_proto_;
};

TEST_F(SuperstructureTest, SysInactive) {
  driver_station_proto_->set_is_sys_active(false);

  intake_group_goal_proto_->set_ground_gear_intake(intake_group::GROUND_GEAR_DROP);
  intake_group_goal_proto_->set_ground_ball_rollers(intake_group::GROUND_BALL_IN);
  intake_group_goal_proto_->set_agitate(true);
  intake_group_goal_proto_->set_magazine_open(true);

  WriteQueues();
  superstructure.Update();

  auto superstructure_output =
      QueueManager::GetInstance().superstructure_output_queue().ReadLastMessage().value();

  EXPECT_EQ(superstructure_output->main_roller_voltage(), 0.0);
  EXPECT_FALSE(superstructure_output->ball_intake_down());
  EXPECT_EQ(superstructure_output->ground_gear_voltage(), 0.0);
  EXPECT_FALSE(superstructure_output->ground_gear_down());
  EXPECT_EQ(superstructure_output->upper_conveyor_voltage(), 0.0);
  EXPECT_EQ(superstructure_output->side_conveyor_voltage(), 0.0);
  EXPECT_FALSE(superstructure_output->magazine_open());
  EXPECT_EQ(superstructure_output->shooter_voltage(), 0.0);
  EXPECT_EQ(superstructure_output->accelerator_voltage(), 0.0);

  auto superstructure_status =
      QueueManager::GetInstance().superstructure_status_queue().ReadLastMessage().value();
  EXPECT_FALSE(superstructure_status->enable_outputs());
}

TEST_F(SuperstructureTest, GearFunctionality) {
  driver_station_proto_->set_is_sys_active(true);

  intake_group_goal_proto_->set_ground_gear_intake(intake_group::GROUND_GEAR_DROP);

  WriteQueues();
  superstructure.Update();

  {
    auto superstructure_output =
        QueueManager::GetInstance().superstructure_output_queue().ReadLastMessage().value();
    auto ground_gear_status =
        QueueManager::GetInstance().ground_gear_status_queue().ReadLastMessage().value();

    EXPECT_EQ(superstructure_output->ground_gear_voltage(),
              c2017::ground_gear_intake::kIntakeVoltage);
    EXPECT_TRUE(superstructure_output->ground_gear_down());
    EXPECT_EQ(ground_gear_status->current_state(), c2017::ground_gear_intake::INTAKING);
  }

  intake_group_goal_proto_->set_ground_gear_intake(intake_group::GROUND_GEAR_NONE);

  // 134 should always count as a spike
  ground_gear_input_proto_->set_current(134);

  WriteQueues();
  superstructure.Update();
  superstructure.Update();

  {
    auto superstructure_output =
        QueueManager::GetInstance().superstructure_output_queue().ReadLastMessage().value();
    auto ground_gear_status =
        QueueManager::GetInstance().ground_gear_status_queue().ReadLastMessage().value();

    EXPECT_EQ(superstructure_output->ground_gear_voltage(),
              c2017::ground_gear_intake::kPickupVoltage);
    EXPECT_FALSE(superstructure_output->ground_gear_down());
    EXPECT_EQ(ground_gear_status->current_state(), c2017::ground_gear_intake::PICKING_UP);
  }

  for (size_t i = 0; i < c2017::ground_gear_intake::kPickupTicks + 1; i++) {
    superstructure.Update();
  }

  {
    auto superstructure_output =
        QueueManager::GetInstance().superstructure_output_queue().ReadLastMessage().value();
    auto ground_gear_status =
        QueueManager::GetInstance().ground_gear_status_queue().ReadLastMessage().value();

    EXPECT_EQ(superstructure_output->ground_gear_voltage(),
              c2017::ground_gear_intake::kCarryVoltage);
    EXPECT_FALSE(superstructure_output->ground_gear_down());
    EXPECT_EQ(ground_gear_status->current_state(), c2017::ground_gear_intake::CARRYING);
  }
}

TEST_F(SuperstructureTest, BothIntakesStayInBox) {
  driver_station_proto_->set_is_sys_active(true);

  intake_group_goal_proto_->set_ground_gear_intake(intake_group::GROUND_GEAR_DROP);
  intake_group_goal_proto_->set_ground_ball_position(intake_group::GROUND_BALL_DOWN);

  WriteQueues();
  superstructure.Update();

  {
    auto superstructure_output =
        QueueManager::GetInstance().superstructure_output_queue().ReadLastMessage().value();

    EXPECT_TRUE(superstructure_output->ground_gear_down());
    EXPECT_FALSE(superstructure_output->ball_intake_down());
  }
}

TEST_F(SuperstructureTest, GearScoreStaysInBox) {
  driver_station_proto_->set_is_sys_active(true);

  intake_group_goal_proto_->set_ground_gear_intake(intake_group::GROUND_GEAR_SCORE);
  intake_group_goal_proto_->set_ground_ball_position(intake_group::GROUND_BALL_DOWN);

  WriteQueues();
  superstructure.Update();

  {
    auto superstructure_output =
        QueueManager::GetInstance().superstructure_output_queue().ReadLastMessage().value();

    EXPECT_FALSE(superstructure_output->ground_gear_down());
    EXPECT_FALSE(superstructure_output->ball_intake_down());
  }
}

TEST_F(SuperstructureTest, ShootingStaysInBox) {
  driver_station_proto_->set_is_sys_active(true);

  shooter_group_goal_proto_->set_wheel(c2017::shooter_group::SHOOT);

  WriteQueues();

  // Update twice to allow shooter status to propagate
  superstructure.Update();
  superstructure.Update();

  {
    auto superstructure_output =
        QueueManager::GetInstance().superstructure_output_queue().ReadLastMessage().value();

    EXPECT_FALSE(superstructure_output->ground_gear_down());
    EXPECT_TRUE(superstructure_output->ball_intake_down());
  }

  intake_group_goal_proto_->set_ground_gear_intake(intake_group::GROUND_GEAR_DROP);

  WriteQueues();
  superstructure.Update();

  {
    auto superstructure_output =
        QueueManager::GetInstance().superstructure_output_queue().ReadLastMessage().value();

    EXPECT_TRUE(superstructure_output->ground_gear_down());
    EXPECT_FALSE(superstructure_output->ball_intake_down());
  }
}

TEST_F(SuperstructureTest, Spinup) {
  driver_station_proto_->set_is_sys_active(true);

  shooter_group_goal_proto_->set_wheel(c2017::shooter_group::SPINUP);

  WriteQueues();

  // Update twice to allow shooter status to propagate
  superstructure.Update();
  superstructure.Update();

  {
    auto superstructure_output =
        QueueManager::GetInstance().superstructure_output_queue().ReadLastMessage().value();

    EXPECT_GT(superstructure_output->shooter_voltage(), 0.0);
    EXPECT_GT(superstructure_output->accelerator_voltage(), 0.0);
    EXPECT_EQ(superstructure_output->upper_conveyor_voltage(), 0.0);
  }
}

TEST_F(SuperstructureTest, SpinupShoot) {
  driver_station_proto_->set_is_sys_active(true);

  double current_position = 0.0;

  shooter_input_proto_->set_shooter_encoder_position(current_position);
  shooter_group_goal_proto_->set_wheel(c2017::shooter_group::BOTH);

  WriteQueues();

  // Update twice to allow shooter status to propagate
  superstructure.Update();
  superstructure.Update();

  // Simulate the shooter moving at a constant fixed speed
  for (size_t i = 0; i < 1000; i++) {
    current_position += c2017::superstructure::kShooterVelocity * 0.005;
    shooter_input_proto_->set_shooter_encoder_position(current_position);
    shooter_input_proto_->set_accelerator_encoder_position(current_position * 0.5);
    WriteQueues();
    superstructure.Update();
  }

  {
    auto shooter_status =
        QueueManager::GetInstance().shooter_status_queue().ReadLastMessage().value();
    auto superstructure_output =
        QueueManager::GetInstance().superstructure_output_queue().ReadLastMessage().value();

    EXPECT_TRUE(shooter_status->at_goal());
    EXPECT_TRUE(shooter_status->currently_running());
    EXPECT_GT(superstructure_output->upper_conveyor_voltage(), 0.0);
  }
}

TEST_F(SuperstructureTest, SpinupManualShoot) {
  driver_station_proto_->set_is_sys_active(true);

  shooter_group_goal_proto_->set_wheel(c2017::shooter_group::SPINUP);

  WriteQueues();

  // Update twice to allow shooter status to propagate
  superstructure.Update();
  superstructure.Update();

  {
    auto superstructure_output =
        QueueManager::GetInstance().superstructure_output_queue().ReadLastMessage().value();

    EXPECT_GT(superstructure_output->shooter_voltage(), 0.0);
    EXPECT_GT(superstructure_output->accelerator_voltage(), 0.0);
  }

  shooter_group_goal_proto_->set_wheel(c2017::shooter_group::SHOOT);

  WriteQueues();
  superstructure.Update();

  {
    auto superstructure_output =
        QueueManager::GetInstance().superstructure_output_queue().ReadLastMessage().value();

    EXPECT_GT(superstructure_output->shooter_voltage(), 0.0);
    EXPECT_GT(superstructure_output->accelerator_voltage(), 0.0);
    EXPECT_GT(superstructure_output->upper_conveyor_voltage(), 0.0);
  }
}

TEST_F(SuperstructureTest, Agitate) {
  driver_station_proto_->set_is_sys_active(true);

  intake_group_goal_proto_->set_agitate(true);

  WriteQueues();
  superstructure.Update();

  {
    auto superstructure_output =
        QueueManager::GetInstance().superstructure_output_queue().ReadLastMessage().value();

    // Should be sending a nonzero side conveyor voltage
    EXPECT_NE(superstructure_output->side_conveyor_voltage(), 0.0);
  }
}

TEST_F(SuperstructureTest, BallIntakeWithConveyor) {
  driver_station_proto_->set_is_sys_active(true);

  intake_group_goal_proto_->set_ground_ball_rollers(intake_group::GROUND_BALL_OUT);

  WriteQueues();
  superstructure.Update();

  {
    auto superstructure_output =
        QueueManager::GetInstance().superstructure_output_queue().ReadLastMessage().value();

    // Should be sending a reverse main roller voltage to spit out
    EXPECT_LT(superstructure_output->main_roller_voltage(), 0.0);
  }

  shooter_group_goal_proto_->set_wheel(c2017::shooter_group::SHOOT);

  WriteQueues();
  superstructure.Update();

  {
    auto superstructure_output =
        QueueManager::GetInstance().superstructure_output_queue().ReadLastMessage().value();

    // Should be forwards, as shooting takes precedence over outtake
    EXPECT_GT(superstructure_output->main_roller_voltage(), 0.0);
  }
}

TEST_F(SuperstructureTest, Climb) {
  driver_station_proto_->set_is_sys_active(true);

  shooter_group_goal_proto_->set_should_climb(true);

  WriteQueues();
  superstructure.Update();
  superstructure.Update();

  {
    auto superstructure_output =
        QueueManager::GetInstance().superstructure_output_queue().ReadLastMessage().value();
    auto superstructure_status =
        QueueManager::GetInstance().superstructure_status_queue().ReadLastMessage().value();

    EXPECT_GT(superstructure_output->accelerator_voltage(), 0.0);
    EXPECT_TRUE(superstructure_status->climbing());

    EXPECT_EQ(superstructure_output->ground_gear_voltage(), c2017::ground_gear_intake::kOuttakeVoltage);
  }
}

TEST_F(SuperstructureTest, CancelClimb) {
  driver_station_proto_->set_is_sys_active(true);

  shooter_group_goal_proto_->set_should_climb(true);

  WriteQueues();
  superstructure.Update();
  superstructure.Update();

  {
    auto superstructure_output =
        QueueManager::GetInstance().superstructure_output_queue().ReadLastMessage().value();
    auto superstructure_status =
        QueueManager::GetInstance().superstructure_status_queue().ReadLastMessage().value();

    EXPECT_GT(superstructure_output->accelerator_voltage(), 0.0);
    EXPECT_TRUE(superstructure_status->climbing());
  }

  shooter_group_goal_proto_->set_should_climb(false);

  WriteQueues();
  superstructure.Update();

  {
    auto superstructure_output =
        QueueManager::GetInstance().superstructure_output_queue().ReadLastMessage().value();
    auto superstructure_status =
        QueueManager::GetInstance().superstructure_status_queue().ReadLastMessage().value();

    EXPECT_EQ(superstructure_output->accelerator_voltage(), 0.0);
    EXPECT_FALSE(superstructure_status->climbing());
  }
}

TEST_F(SuperstructureTest, ShootToClimb) {
  driver_station_proto_->set_is_sys_active(true);

  shooter_group_goal_proto_->set_wheel(c2017::shooter_group::BOTH);

  WriteQueues();

  superstructure.Update();
  superstructure.Update();

  shooter_group_goal_proto_->set_should_climb(true);

  WriteQueues();
  superstructure.Update();

  {
    auto superstructure_output =
        QueueManager::GetInstance().superstructure_output_queue().ReadLastMessage().value();
    auto superstructure_status =
        QueueManager::GetInstance().superstructure_status_queue().ReadLastMessage().value();

    EXPECT_GT(superstructure_output->accelerator_voltage(), 0.0);
    EXPECT_TRUE(superstructure_status->climbing());
  }
}

}  // namespace c2017
