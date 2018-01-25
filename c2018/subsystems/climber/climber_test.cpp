#include "c2018/subsystems/climber/climber.h"
#include "gtest/gtest.h"
#include "muan/queues/queue_manager.h"

class ClimberTest : public ::testing::Test {
 public:
  ClimberTest() {}

  void Update() { climber_.Update(); }

  void ReadMessages() {
    output_queue_.ReadLastMessage(&climber_output_proto_);
    status_queue_.ReadLastMessage(&climber_status_proto_);
  }

  void WriteMessages() {
    climber_input_queue_->WriteMessage(climber_input_proto_);
    climber_goal_queue_->WriteMessage(climber_goal_proto_);
    driver_station_queue_->WriteMessage(driver_station_proto_);
  }

  void SetGoals(bool batter_down, c2018::climber::Goal enum_goal, bool enabled) {
    climber_goal_proto_->set_put_down_batter(batter_down);
    climber_goal_proto_->set_climber_goal(enum_goal);
    ds_status_reader_->set_is_sys_active(enabled);
  }

  void SetInput(double position) { climber_input_proto_->set_position(position); }

  // QUEUES & READERS
  muan::wpilib::DriverStationQueue* driver_station_queue_ =
      muan::queues::QueueManager<muan::wpilib::DriverStationProto>::Fetch();

  c2018::climber::ClimberGoalQueue* climber_goal_queue_ =
      muan::queues::QueueManager<c2018::climber::ClimberGoalProto>::Fetch();

  c2018::climber::ClimberInputQueue* climber_input_queue_ =
      muan::queues::QueueManager<c2018::climber::ClimberInputProto>::Fetch();

  c2018::climber::ClimberStatusQueue::QueueReader climber_status_queue_ =
      muan::queues::QueueManager<c2018::climber::ClimberStatusProto>::Fetch()->MakeReader();

  c2018::climber::ClimberOutputQueue::QueueReader climber_output_queue_ =
      muan::queues::QueueManager<c2018::climber::ClimberOutputProto>::Fetch()->MakeReader();

  // PROTOS
  muan::wpilib::DriverStationProto driver_station_proto_;
  c2018::climber::ClimberGoalProto climber_goal_proto_;
  c2018::climber::ClimberInputProto climber_input_proto_;
  c2018::climber::ClimberStatusProto climber_status_proto_;
  c2018::climber::ClimberOutputProto climber_output_proto_;

 private:
  c2018::climber::Climber climber_;
};

TEST_F(ClimberTest, Idle) {
  SetGoals(false, c2018::climber::Goal::NONE, true);
  SetInput(0.0);

  WriteMessages();
  Update();
  ReadMessages();

  EXPECT_EQ(climber_output_proto_->voltage(), 0.);
  EXPECT_FALSE(climber_output_proto_->release_solenoid());
  EXPECT_EQ(climber_status_proto_->climber_state(), c2018::climber::State::IDLE);
  EXPECT_EQ(climber_status_proto_->observed_velocity(), 0.);
  EXPECT_EQ(climber_status_proto_->observed_height(), 0.);
}

TEST_F(ClimberTest, Approach) {
  SetGoals(true, c2018::climber::Goal::APPROACHING, true);
  SetInput(0.0);

  WriteMessages();
  Update();
  ReadMessages();

  // TEST VALUES
  EXPECT_EQ(climber_output_proto_->voltage(), 0.);
  EXPECT_TRUE(climber_output_proto_->release_solenoid());
  EXPECT_EQ(climber_status_proto_->climber_state(), c2018::climber::State::APPROACH);
  EXPECT_EQ(climber_status_proto_->observed_velocity(), 0.);
  EXPECT_EQ(climber_status_proto_->observed_height(), 0.);
}

TEST_F(ClimberTest, Climb) {
  SetGoals(true, c2018::climber::Goal::CLIMBING, true);

  WriteMessages();

  // UPDATING CLIMBER
  for (int i = 0; i < 200; i++) {
    SetInput(i);
    climber_input_queue_->WriteMessage(climber_input_proto_);
    Update();
  }

  // CHECKING HALFWAY THROUGH CLIMB
  ReadMessages();
  EXPECT_EQ(climber_status_proto_->climber_state(), c2018::climber::State::CLIMB);

  // UPDATING FOR LONGER
  for (int i = 0; i < 3000; i++) {
    SetInput(i);
    climber_input_queue_->WriteMessage(climber_input_proto_);
    Update();
  }

  ReadMessages();

  // TESTING OUTPUTS
  EXPECT_EQ(climber_output_proto_->voltage(), 0.);
  EXPECT_TRUE(climber_output_proto_->release_solenoid());
  EXPECT_EQ(climber_status_proto_->climber_state(), c2018::climber::State::DONE);
  EXPECT_EQ(climber_status_proto_->observed_velocity(), 0.);
  EXPECT_EQ(climber_status_proto_->observed_height(), 0.);
}

TEST_F(ClimberTest, OutputsNotEnabled) {
  SetGoals(false, c2018::climber::Goal::NONE, false);

  WriteMessages();

  Update();

  ReadMessages();

  EXPECT_EQ(climber_output_proto_->voltage(), 0.);
  EXPECT_FALSE(climber_output_proto_->release_solenoid());
  EXPECT_EQ(climber_status_proto_->climber_state(), c2018::climber::State::IDLE);
  EXPECT_EQ(climber_status_proto_->observed_velocity(), 0.);
  EXPECT_EQ(climber_status_proto_->observed_height(), 0.);
}
