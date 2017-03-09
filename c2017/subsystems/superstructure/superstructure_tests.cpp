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
  intake_group_goal_proto_->set_ground_ball_position(intake_group::GROUND_BALL_UP);
  intake_group_goal_proto_->set_ground_gear_intake(intake_group::GROUND_GEAR_DROP);

  WriteQueues();

  superstructure.Update();

  auto superstructure_output = QueueManager::GetInstance().superstructure_output_queue().ReadLastMessage();

  ASSERT_TRUE(superstructure_output);

  EXPECT_NE(superstructure_output.value()->ground_gear_voltage(), 0);
  EXPECT_TRUE(superstructure_output.value()->ground_gear_down());
  EXPECT_FALSE(superstructure_output.value()->ball_intake_down());

  Reset();
}

TEST_F(SuperstructureTest, GroundGearScoring) {
  ds->set_mode(RobotMode::TELEOP);
  shooter_status_proto_->set_at_goal(true);

  intake_group_goal_proto_->set_ground_ball_position(intake_group::GROUND_BALL_UP);
  intake_group_goal_proto_->set_hp_load_type(intake_group::HP_LOAD_NONE);
  intake_group_goal_proto_->set_ground_ball_rollers(intake_group::GROUND_BALL_NONE);
  intake_group_goal_proto_->set_ground_gear_intake(intake_group::GROUND_GEAR_SCORE);

  WriteQueues();

  superstructure.Update();

  auto superstructure_output = QueueManager::GetInstance().superstructure_output_queue().ReadLastMessage();

  ASSERT_TRUE(superstructure_output);
  EXPECT_NEAR(superstructure_output.value()->ground_gear_voltage(), -12, 1e-4);
  EXPECT_FALSE(superstructure_output.value()->ground_gear_down());

  Reset();
}

TEST_F(SuperstructureTest, BallIntaking) {
  ds->set_mode(RobotMode::TELEOP);
  shooter_status_proto_->set_at_goal(true);

  intake_group_goal_proto_->set_ground_ball_position(intake_group::GROUND_BALL_DOWN);
  intake_group_goal_proto_->set_hp_load_type(intake_group::HP_LOAD_NONE);
  intake_group_goal_proto_->set_ground_ball_rollers(intake_group::GROUND_BALL_IN);
  intake_group_goal_proto_->set_ground_gear_intake(intake_group::GROUND_GEAR_NONE);

  WriteQueues();

  superstructure.Update();

  auto superstructure_output = QueueManager::GetInstance().superstructure_output_queue().ReadLastMessage();

  ASSERT_TRUE(superstructure_output);
  EXPECT_TRUE(superstructure_output.value()->ball_intake_down());
  EXPECT_FALSE(superstructure_output.value()->ground_gear_down());
  EXPECT_NEAR(superstructure_output.value()->main_roller_voltage(), 8, 1e-4);

  Reset();
}

TEST_F(SuperstructureTest, BallReverse) {
  shooter_status_proto_->set_at_goal(true);

  ds->set_mode(RobotMode::TELEOP);
  intake_group_goal_proto_->set_ground_ball_position(intake_group::GROUND_BALL_DOWN);
  intake_group_goal_proto_->set_hp_load_type(intake_group::HP_LOAD_NONE);
  intake_group_goal_proto_->set_ground_ball_rollers(intake_group::GROUND_BALL_OUT);
  intake_group_goal_proto_->set_ground_gear_intake(intake_group::GROUND_GEAR_NONE);

  WriteQueues();
  superstructure.Update();

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
  intake_group_goal_proto_->set_ground_ball_position(intake_group::GROUND_BALL_DOWN);
  intake_group_goal_proto_->set_hp_load_type(intake_group::HP_LOAD_NONE);
  intake_group_goal_proto_->set_ground_ball_rollers(intake_group::GROUND_BALL_IN);
  intake_group_goal_proto_->set_ground_gear_intake(intake_group::GROUND_GEAR_NONE);
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
  EXPECT_NEAR(superstructure_output.value()->upper_conveyor_voltage(), 12, 1e-4);
  Reset();
}

TEST_F(SuperstructureTest, SpinupShootHopper) {
  ds->set_mode(RobotMode::TELEOP);
  shooter_status_proto_->set_at_goal(true);
  shooter_status_proto_->set_currently_running(true);
  intake_group_goal_proto_->set_ground_ball_position(intake_group::GROUND_BALL_DOWN);
  intake_group_goal_proto_->set_hp_load_type(intake_group::HP_LOAD_NONE);
  intake_group_goal_proto_->set_ground_ball_rollers(intake_group::GROUND_BALL_IN);
  intake_group_goal_proto_->set_ground_gear_intake(intake_group::GROUND_GEAR_NONE);
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
  EXPECT_NEAR(superstructure_output.value()->upper_conveyor_voltage(), 12, 1e-4);
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
  EXPECT_TRUE(superstructure_output.value()->ball_intake_down());

  // Update it again to make sure it stays open
  superstructure.Update();

  superstructure_output = QueueManager::GetInstance().superstructure_output_queue().ReadLastMessage();

  ASSERT_TRUE(superstructure_output);
  EXPECT_TRUE(superstructure_output.value()->gear_shutter_open());
  EXPECT_TRUE(superstructure_output.value()->ball_intake_down());

  // And then make sure it all closes when it should
  intake_group_goal_proto_->set_score_hp_gear(false);
  WriteQueues();
  superstructure.Update();

  superstructure_output = QueueManager::GetInstance().superstructure_output_queue().ReadLastMessage();

  ASSERT_TRUE(superstructure_output);
  EXPECT_FALSE(superstructure_output.value()->gear_shutter_open());
  EXPECT_FALSE(superstructure_output.value()->ball_intake_down());

  Reset();
}

TEST_F(SuperstructureTest, Disabled) {
  ds->set_mode(RobotMode::DISABLED);

  // Just make a bunch of things spin, since we're testing disabled.
  shooter_status_proto_->set_at_goal(true);
  shooter_status_proto_->set_currently_running(true);
  intake_group_goal_proto_->set_ground_ball_position(intake_group::GROUND_BALL_DOWN);
  intake_group_goal_proto_->set_hp_load_type(intake_group::HP_LOAD_NONE);
  intake_group_goal_proto_->set_ground_ball_rollers(intake_group::GROUND_BALL_IN);
  intake_group_goal_proto_->set_ground_gear_intake(intake_group::GROUND_GEAR_NONE);
  shooter_group_goal_proto_->set_wheel(shooter_group::BOTH);
  shooter_group_goal_proto_->set_position(shooter_group::HOPPER);

  WriteQueues();
  superstructure.Update();

  auto superstructure_output = QueueManager::GetInstance().superstructure_output_queue().ReadLastMessage();

  ASSERT_TRUE(superstructure_output);
  EXPECT_NEAR(superstructure_output.value()->main_roller_voltage(), 0, 1e-4);
  EXPECT_NEAR(superstructure_output.value()->ground_gear_voltage(), 0, 1e-4);
  EXPECT_NEAR(superstructure_output.value()->upper_conveyor_voltage(), 0, 1e-4);
  EXPECT_NEAR(superstructure_output.value()->side_conveyor_voltage(), 0, 1e-4);
  EXPECT_NEAR(superstructure_output.value()->shooter_voltage(), 0, 1e-4);
  Reset();
}

TEST_F(SuperstructureTest, Brownout) {
  ds->set_mode(RobotMode::TELEOP);
  ds->set_brownout(true);

  // Just make a bunch of things spin, since we're testing brownout.
  shooter_status_proto_->set_at_goal(true);
  shooter_status_proto_->set_currently_running(true);
  intake_group_goal_proto_->set_ground_ball_position(intake_group::GROUND_BALL_DOWN);
  intake_group_goal_proto_->set_hp_load_type(intake_group::HP_LOAD_NONE);
  intake_group_goal_proto_->set_ground_ball_rollers(intake_group::GROUND_BALL_IN);
  intake_group_goal_proto_->set_ground_gear_intake(intake_group::GROUND_GEAR_NONE);
  shooter_group_goal_proto_->set_wheel(shooter_group::BOTH);
  shooter_group_goal_proto_->set_position(shooter_group::HOPPER);

  WriteQueues();
  superstructure.Update();

  auto superstructure_output = QueueManager::GetInstance().superstructure_output_queue().ReadLastMessage();

  ASSERT_TRUE(superstructure_output);
  EXPECT_NEAR(superstructure_output.value()->main_roller_voltage(), 0, 1e-4);
  EXPECT_NEAR(superstructure_output.value()->ground_gear_voltage(), 0, 1e-4);
  EXPECT_NEAR(superstructure_output.value()->upper_conveyor_voltage(), 0, 1e-4);
  EXPECT_NEAR(superstructure_output.value()->side_conveyor_voltage(), 0, 1e-4);
  EXPECT_NEAR(superstructure_output.value()->shooter_voltage(), 0, 1e-4);
  Reset();
}

TEST_F(SuperstructureTest, ThinkInsideTheBox) {
  // If you tell ball and gear intake to go down, only gear intake should move.
  intake_group_goal_proto_->set_ground_ball_position(intake_group::GROUND_BALL_UP);
  intake_group_goal_proto_->set_ground_gear_intake(intake_group::GROUND_GEAR_DROP);
  ds->set_mode(RobotMode::TELEOP);

  WriteQueues();
  superstructure.Update();

  auto superstructure_output = QueueManager::GetInstance().superstructure_output_queue().ReadLastMessage();

  ASSERT_TRUE(superstructure_output);

  EXPECT_TRUE(superstructure_output.value()->ground_gear_down());
  EXPECT_FALSE(superstructure_output.value()->ball_intake_down());

  Reset();

  // Also, when you're trying to HP-gear score and the gear intake is down, it should put it back up so that
  // it can put the ball intake down
  intake_group_goal_proto_->set_ground_gear_intake(intake_group::GROUND_GEAR_DROP);
  ds->set_mode(RobotMode::TELEOP);
  WriteQueues();
  superstructure.Update();
  intake_group_goal_proto_->set_score_hp_gear(true);
  WriteQueues();
  superstructure.Update();

  superstructure_output = QueueManager::GetInstance().superstructure_output_queue().ReadLastMessage();

  ASSERT_TRUE(superstructure_output);

  EXPECT_FALSE(superstructure_output.value()->ground_gear_down());
  EXPECT_TRUE(superstructure_output.value()->ball_intake_down());
  EXPECT_TRUE(superstructure_output.value()->gear_shutter_open());
}

}  // namespace c2017
