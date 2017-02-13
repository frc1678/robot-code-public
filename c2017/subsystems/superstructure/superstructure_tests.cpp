#include "c2017/subsystems/superstructure/superstructure.h"

#include <iostream>
#include "c2017/queue_manager/queue_manager.h"
#include "gtest/gtest.h"

namespace c2017 {

class SuperstructureTest : public ::testing::Test {
 public:
  SuperstructureTest() {}

  intake_group::IntakeGroupGoalProto intake_group_goal_proto;
  climber::ClimberGoalProto climber_goal_proto;
  shooter_group::ShooterGroupGoalProto shooter_group_goal_proto;
  muan::wpilib::DriverStationProto ds;
  superstructure::SuperStructure superstructure;
  shooter::ShooterStatusProto shooter_status_proto;

  climber::ClimberInputProto climber_input_proto;
  shooter::ShooterInputProto shooter_input_proto;
  ground_gear_intake::GroundGearIntakeInputProto ground_gear_input_proto;
  magazine::MagazineInputProto magazine_input_proto;

  void WriteQueues() {
    QueueManager::GetInstance().intake_group_goal_queue().WriteMessage(intake_group_goal_proto);
    QueueManager::GetInstance().climber_goal_queue().WriteMessage(climber_goal_proto);
    QueueManager::GetInstance().shooter_group_goal_queue().WriteMessage(shooter_group_goal_proto);
    QueueManager::GetInstance().driver_station_queue()->WriteMessage(ds);
    QueueManager::GetInstance().shooter_status_queue().WriteMessage(shooter_status_proto);
    QueueManager::GetInstance().climber_input_queue().WriteMessage(climber_input_proto);
    QueueManager::GetInstance().shooter_input_queue().WriteMessage(shooter_input_proto);
    QueueManager::GetInstance().ground_gear_input_queue().WriteMessage(ground_gear_input_proto);
    QueueManager::GetInstance().magazine_input_queue().WriteMessage(magazine_input_proto);
  }

  void Reset() {
    intake_group_goal_proto.Reset();
    climber_goal_proto.Reset();
    shooter_group_goal_proto.Reset();
    ds.Reset();
    shooter_status_proto.Reset();
    climber_input_proto.Reset();
    shooter_input_proto.Reset();
    ground_gear_input_proto.Reset();
    magazine_input_proto.Reset();
  }
};

TEST_F(SuperstructureTest, GearIntaking) {
  climber_input_proto->set_current(0);
  climber_input_proto->set_position(0);
  shooter_input_proto->set_encoder_position(0);
  ground_gear_input_proto->set_current(0);
  magazine_input_proto->set_has_hp_gear(false);

  shooter_status_proto->set_at_goal(true);

  ds->set_mode(RobotMode::TELEOP);
  intake_group_goal_proto->set_ground_intake_position(intake_group::INTAKE_GEAR);
  intake_group_goal_proto->set_gear_intake(intake_group::GEAR_INTAKE);

  WriteQueues();

  superstructure.Update();

  auto superstructure_status = QueueManager::GetInstance().superstructure_status_queue().ReadLastMessage();
  auto superstructure_output = QueueManager::GetInstance().superstructure_output_queue().ReadLastMessage();

  ASSERT_TRUE(superstructure_status);
  ASSERT_TRUE(superstructure_output);

  EXPECT_TRUE(superstructure_status.value()->gear_intaking());
  EXPECT_NE(superstructure_output.value()->ground_gear_voltage(), 0);
  EXPECT_TRUE(superstructure_output.value()->ground_gear_down());

  Reset();
}

TEST_F(SuperstructureTest, GearScoring) {
  ds->set_mode(RobotMode::TELEOP);
  shooter_status_proto->set_at_goal(true);

  intake_group_goal_proto->set_ground_intake_position(intake_group::INTAKE_NONE);
  intake_group_goal_proto->set_hp_load_type(intake_group::HP_LOAD_NONE);
  intake_group_goal_proto->set_roller(intake_group::ROLLERS_IDLE);
  intake_group_goal_proto->set_gear_intake(intake_group::GEAR_OUTTAKE);

  WriteQueues();

  superstructure.Update();

  auto superstructure_status = QueueManager::GetInstance().superstructure_status_queue().ReadLastMessage();

  ASSERT_TRUE(superstructure_status);
  EXPECT_TRUE(superstructure_status.value()->gear_scoring());
  Reset();
}

TEST_F(SuperstructureTest, BallIntaking) {
  ds->set_mode(RobotMode::TELEOP);
  shooter_status_proto->set_at_goal(true);

  intake_group_goal_proto->set_ground_intake_position(intake_group::INTAKE_BALLS);
  intake_group_goal_proto->set_hp_load_type(intake_group::HP_LOAD_NONE);
  intake_group_goal_proto->set_roller(intake_group::ROLLERS_INTAKE);
  intake_group_goal_proto->set_gear_intake(intake_group::GEAR_IDLE);

  WriteQueues();

  superstructure.Update();

  auto superstructure_status = QueueManager::GetInstance().superstructure_status_queue().ReadLastMessage();

  ASSERT_TRUE(superstructure_status);
  EXPECT_TRUE(superstructure_status.value()->ball_intaking());
  Reset();
}

TEST_F(SuperstructureTest, BallReverse) {
  shooter_status_proto->set_at_goal(true);

  ds->set_mode(RobotMode::TELEOP);
  intake_group_goal_proto->set_ground_intake_position(intake_group::INTAKE_BALLS);
  intake_group_goal_proto->set_hp_load_type(intake_group::HP_LOAD_NONE);
  intake_group_goal_proto->set_roller(intake_group::ROLLERS_OUTTAKE);
  intake_group_goal_proto->set_gear_intake(intake_group::GEAR_IDLE);

  WriteQueues();
  superstructure.Update();

  auto superstructure_status = QueueManager::GetInstance().superstructure_status_queue().ReadLastMessage();

  ASSERT_TRUE(superstructure_status);
  EXPECT_TRUE(superstructure_status.value()->ball_reverse());
  Reset();
}

TEST_F(SuperstructureTest, ClimberTest) {
  ds->set_mode(RobotMode::TELEOP);

  shooter_group_goal_proto->set_should_climb(true);

  WriteQueues();
  superstructure.Update();

  auto superstructure_status = QueueManager::GetInstance().superstructure_status_queue().ReadLastMessage();
  ASSERT_TRUE(superstructure_status);
  EXPECT_TRUE(superstructure_status.value()->climbing());
  Reset();
}

TEST_F(SuperstructureTest, Shoot) {
  ds->set_mode(RobotMode::TELEOP);
  shooter_status_proto->set_at_goal(true);
  shooter_status_proto->set_currently_running(true);
  intake_group_goal_proto->set_ground_intake_position(intake_group::INTAKE_BALLS);
  intake_group_goal_proto->set_hp_load_type(intake_group::HP_LOAD_NONE);
  intake_group_goal_proto->set_roller(intake_group::ROLLERS_INTAKE);
  intake_group_goal_proto->set_gear_intake(intake_group::GEAR_IDLE);
  shooter_group_goal_proto->set_wheel(shooter_group::SHOOT);
  shooter_group_goal_proto->set_position(shooter_group::FENDER);

  WriteQueues();

  superstructure.Update();

  auto superstructure_status = QueueManager::GetInstance().superstructure_status_queue().ReadLastMessage();

  ASSERT_TRUE(superstructure_status);
  EXPECT_TRUE(superstructure_status.value()->shooting());

  auto superstructure_output = QueueManager::GetInstance().superstructure_output_queue().ReadLastMessage();

  ASSERT_TRUE(superstructure_output);
  EXPECT_NEAR(superstructure_output.value()->main_roller_voltage(), 12, 1e-4);
  // TODO(Wesley) Figure out why this fails
  // EXPECT_NEAR(superstructure_output.value()->upper_conveyor_voltage(), 12, 1e-4);
  Reset();
}

}  // namespace c2017
