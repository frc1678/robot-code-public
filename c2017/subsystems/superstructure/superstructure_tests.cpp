#include "c2017/subsystems/superstructure/superstructure.h"

#include <iostream>
#include "c2017/queue_manager/queue_manager.h"
#include "gtest/gtest.h"

namespace c2017 {

class SuperstructureTest : public ::testing::Test {
 public:
  SuperstructureTest() {}

  intake_group::IntakeGroupGoalProto intake_group_goal_proto_;
  climber::ClimberGoalProto climber_goal_proto_;
  shooter_group::ShooterGroupGoalProto shooter_group_goal_proto_;
  muan::wpilib::DriverStationProto ds;
  superstructure::SuperStructure superstructure;
  shooter::ShooterStatusProto shooter_status_proto_;

  climber::ClimberInputProto climber_input_proto_;
  shooter::ShooterInputProto shooter_input_proto_;
  ground_gear_intake::GroundGearIntakeInputProto ground_gear_input_proto_;
  magazine::MagazineInputProto magazine_input_proto_;

  void WriteQueues() {
    QueueManager::GetInstance().intake_group_goal_queue().WriteMessage(intake_group_goal_proto_);
    QueueManager::GetInstance().climber_goal_queue().WriteMessage(climber_goal_proto_);
    QueueManager::GetInstance().shooter_group_goal_queue().WriteMessage(shooter_group_goal_proto_);
    QueueManager::GetInstance().driver_station_queue()->WriteMessage(ds);
    QueueManager::GetInstance().shooter_status_queue().WriteMessage(shooter_status_proto_);
    QueueManager::GetInstance().climber_input_queue().WriteMessage(climber_input_proto_);
    QueueManager::GetInstance().shooter_input_queue().WriteMessage(shooter_input_proto_);
    QueueManager::GetInstance().ground_gear_input_queue().WriteMessage(ground_gear_input_proto_);
    QueueManager::GetInstance().magazine_input_queue().WriteMessage(magazine_input_proto_);
  }

  void Reset() {
    intake_group_goal_proto_.Reset();
    climber_goal_proto_.Reset();
    shooter_group_goal_proto_.Reset();
    ds.Reset();
    shooter_status_proto_.Reset();
    climber_input_proto_.Reset();
    shooter_input_proto_.Reset();
    ground_gear_input_proto_.Reset();
    magazine_input_proto_.Reset();
  }
};

TEST_F(SuperstructureTest, GroundGearIntaking) {
  climber_input_proto_->set_current(0);
  climber_input_proto_->set_position(0);
  shooter_input_proto_->set_encoder_position(0);
  ground_gear_input_proto_->set_current(0);
  magazine_input_proto_->set_has_hp_gear(false);

  shooter_status_proto_->set_at_goal(true);

  ds->set_mode(RobotMode::TELEOP);
  intake_group_goal_proto_->set_ground_intake_position(intake_group::INTAKE_GEAR);
  intake_group_goal_proto_->set_gear_intake(intake_group::GEAR_INTAKE);

  WriteQueues();

  superstructure.Update();

  auto superstructure_status = QueueManager::GetInstance().superstructure_status_queue().ReadLastMessage();
  auto superstructure_output = QueueManager::GetInstance().superstructure_output_queue().ReadLastMessage();

  ASSERT_TRUE(superstructure_status);
  ASSERT_TRUE(superstructure_output);

  EXPECT_TRUE(superstructure_status.value()->gear_intaking());
  EXPECT_NE(superstructure_output.value()->ground_gear_voltage(), 0);
  EXPECT_TRUE(superstructure_output.value()->ground_gear_down());
  EXPECT_FALSE(superstructure_output.value()->ball_intake_down());

  Reset();
}

TEST_F(SuperstructureTest, GroundGearScoring) {
  ds->set_mode(RobotMode::TELEOP);
  shooter_status_proto_->set_at_goal(true);

  intake_group_goal_proto_->set_ground_intake_position(intake_group::INTAKE_NONE);
  intake_group_goal_proto_->set_hp_load_type(intake_group::HP_LOAD_NONE);
  intake_group_goal_proto_->set_roller(intake_group::ROLLERS_IDLE);
  intake_group_goal_proto_->set_gear_intake(intake_group::GEAR_OUTTAKE);

  WriteQueues();

  superstructure.Update();

  auto superstructure_status = QueueManager::GetInstance().superstructure_status_queue().ReadLastMessage();

  ASSERT_TRUE(superstructure_status);
  EXPECT_TRUE(superstructure_status.value()->gear_scoring());

  auto superstructure_output = QueueManager::GetInstance().superstructure_output_queue().ReadLastMessage();

  ASSERT_TRUE(superstructure_output);
  EXPECT_NEAR(superstructure_output.value()->ground_gear_voltage(), -12, 1e-4);
  EXPECT_FALSE(superstructure_output.value()->ground_gear_down());

  Reset();
}

TEST_F(SuperstructureTest, BallIntaking) {
  ds->set_mode(RobotMode::TELEOP);
  shooter_status_proto_->set_at_goal(true);

  intake_group_goal_proto_->set_ground_intake_position(intake_group::INTAKE_BALLS);
  intake_group_goal_proto_->set_hp_load_type(intake_group::HP_LOAD_NONE);
  intake_group_goal_proto_->set_roller(intake_group::ROLLERS_INTAKE);
  intake_group_goal_proto_->set_gear_intake(intake_group::GEAR_IDLE);

  WriteQueues();

  superstructure.Update();

  auto superstructure_status = QueueManager::GetInstance().superstructure_status_queue().ReadLastMessage();

  ASSERT_TRUE(superstructure_status);
  EXPECT_TRUE(superstructure_status.value()->ball_intaking());

  auto superstructure_output = QueueManager::GetInstance().superstructure_output_queue().ReadLastMessage();

  ASSERT_TRUE(superstructure_output);
  EXPECT_TRUE(superstructure_output.value()->ball_intake_down());
  // TODO(Wesley) Figure out why this is not working:
  // EXPECT_FALSE(superstructure_output.value()->ground_gear_down());
  EXPECT_NEAR(superstructure_output.value()->main_roller_voltage(), 8, 1e-4);

  Reset();
}

TEST_F(SuperstructureTest, BallReverse) {
  shooter_status_proto_->set_at_goal(true);

  ds->set_mode(RobotMode::TELEOP);
  intake_group_goal_proto_->set_ground_intake_position(intake_group::INTAKE_BALLS);
  intake_group_goal_proto_->set_hp_load_type(intake_group::HP_LOAD_NONE);
  intake_group_goal_proto_->set_roller(intake_group::ROLLERS_OUTTAKE);
  intake_group_goal_proto_->set_gear_intake(intake_group::GEAR_IDLE);

  WriteQueues();
  superstructure.Update();

  auto superstructure_status = QueueManager::GetInstance().superstructure_status_queue().ReadLastMessage();

  ASSERT_TRUE(superstructure_status);
  EXPECT_TRUE(superstructure_status.value()->ball_reverse());

  auto superstructure_output = QueueManager::GetInstance().superstructure_output_queue().ReadLastMessage();

  ASSERT_TRUE(superstructure_output);
  EXPECT_NEAR(superstructure_output.value()->main_roller_voltage(), -8, 1e-4);
  Reset();
}

TEST_F(SuperstructureTest, ClimberTest) {
  ds->set_mode(RobotMode::TELEOP);

  shooter_group_goal_proto_->set_should_climb(true);

  WriteQueues();
  superstructure.Update();

  auto superstructure_status = QueueManager::GetInstance().superstructure_status_queue().ReadLastMessage();
  ASSERT_TRUE(superstructure_status);
  EXPECT_TRUE(superstructure_status.value()->climbing());
  Reset();
}

TEST_F(SuperstructureTest, SpinupShootFender) {
  ds->set_mode(RobotMode::TELEOP);
  shooter_status_proto_->set_at_goal(true);
  shooter_status_proto_->set_currently_running(true);
  intake_group_goal_proto_->set_ground_intake_position(intake_group::INTAKE_BALLS);
  intake_group_goal_proto_->set_hp_load_type(intake_group::HP_LOAD_NONE);
  intake_group_goal_proto_->set_roller(intake_group::ROLLERS_INTAKE);
  intake_group_goal_proto_->set_gear_intake(intake_group::GEAR_IDLE);
  shooter_group_goal_proto_->set_wheel(shooter_group::BOTH);
  shooter_group_goal_proto_->set_position(shooter_group::FENDER);

  WriteQueues();

  superstructure.Update();

  auto superstructure_status = QueueManager::GetInstance().superstructure_status_queue().ReadLastMessage();

  ASSERT_TRUE(superstructure_status);
  EXPECT_TRUE(superstructure_status.value()->shooting());

  auto superstructure_output = QueueManager::GetInstance().superstructure_output_queue().ReadLastMessage();

  ASSERT_TRUE(superstructure_output);
  EXPECT_NEAR(superstructure_output.value()->main_roller_voltage(), 12, 1e-4);
  EXPECT_FALSE(superstructure_output.value()->shooter_hood_up());
  // TODO(Wesley) Figure out why this fails
  // EXPECT_NEAR(superstructure_output.value()->upper_conveyor_voltage(), 12, 1e-4);
  Reset();
}

TEST_F(SuperstructureTest, SpinupShootHopper) {
  ds->set_mode(RobotMode::TELEOP);
  shooter_status_proto_->set_at_goal(true);
  shooter_status_proto_->set_currently_running(true);
  intake_group_goal_proto_->set_ground_intake_position(intake_group::INTAKE_BALLS);
  intake_group_goal_proto_->set_hp_load_type(intake_group::HP_LOAD_NONE);
  intake_group_goal_proto_->set_roller(intake_group::ROLLERS_INTAKE);
  intake_group_goal_proto_->set_gear_intake(intake_group::GEAR_IDLE);
  shooter_group_goal_proto_->set_wheel(shooter_group::BOTH);
  shooter_group_goal_proto_->set_position(shooter_group::HOPPER);

  WriteQueues();

  superstructure.Update();

  auto superstructure_status = QueueManager::GetInstance().superstructure_status_queue().ReadLastMessage();

  ASSERT_TRUE(superstructure_status);
  EXPECT_TRUE(superstructure_status.value()->shooting());

  auto superstructure_output = QueueManager::GetInstance().superstructure_output_queue().ReadLastMessage();

  ASSERT_TRUE(superstructure_output);
  EXPECT_NEAR(superstructure_output.value()->main_roller_voltage(), 12, 1e-4);
  EXPECT_TRUE(superstructure_output.value()->shooter_hood_up());
  // TODO(Wesley) Figure out why this fails
  // EXPECT_NEAR(superstructure_output.value()->upper_conveyor_voltage(), 12, 1e-4);
  Reset();
}

TEST_F(SuperstructureTest, HPGearScoring) {
  ds->set_mode(RobotMode::TELEOP);
  shooter_status_proto_->set_at_goal(true);

  intake_group_goal_proto_->set_score_hp_gear(true);

  WriteQueues();

  superstructure.Update();

  auto superstructure_output = QueueManager::GetInstance().superstructure_output_queue().ReadLastMessage();

  ASSERT_TRUE(superstructure_output);
  EXPECT_TRUE(superstructure_output.value()->gear_shutter_open());

  Reset();
}

}  // namespace c2017
