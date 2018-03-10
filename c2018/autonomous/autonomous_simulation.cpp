#include <thread>
#include <string>
#include "c2018/autonomous/autonomous_runner.h"
#include "gtest/gtest.h"
#include "third_party/frc971/control_loops/drivetrain/drivetrain_simulation.h"
#include "muan/queues/queue_manager.h"
#include "c2018/subsystems/drivetrain/drivetrain_base.h"
#include "c2018/subsystems/drivetrain/drivetrain_dog_motor_plant.h"

using c2018::drivetrain::GetDrivetrainConfig;
using c2018::drivetrain::MakeDrivetrainPlant;
using frc971::control_loops::drivetrain::GoalProto;
using frc971::control_loops::drivetrain::InputProto;
using frc971::control_loops::drivetrain::OutputProto;
using frc971::control_loops::drivetrain::StatusProto;
using frc971::control_loops::drivetrain::testing::DrivetrainTest;
using muan::wpilib::DriverStationProto;
using muan::wpilib::GameSpecificStringProto;
using muan::queues::QueueManager;

class C2018DrivetrainTest : public DrivetrainTest {
 public:
  C2018DrivetrainTest()
      : DrivetrainTest(GetDrivetrainConfig(), MakeDrivetrainPlant()) {}

  void RunWithString(std::string gss) {
    DriverStationProto ds_proto;
    ds_proto->set_mode(RobotMode::AUTONOMOUS);
    enable_mode_ = RobotMode::AUTONOMOUS;
    QueueManager<DriverStationProto>::Fetch()->WriteMessage(ds_proto);
    GameSpecificStringProto gss_proto;
    gss_proto->set_code(gss);
    QueueManager<GameSpecificStringProto>::Fetch()->WriteMessage(gss_proto);
    RunIteration();
    c2018::autonomous::AutonomousRunner autonomous;
    std::thread auto_thread(autonomous);
    RunForTime(std::chrono::seconds(15));
    ASSERT_TRUE(status_queue_->ReadLastMessage());
    ds_proto->set_mode(RobotMode::DISABLED);
    enable_mode_ = RobotMode::DISABLED;
    QueueManager<DriverStationProto>::Fetch()->WriteMessage(ds_proto);
    RunForTime(std::chrono::seconds(2));
    auto_thread.join();
  }
};

TEST_F(C2018DrivetrainTest, AutoRR) {
  BeginLogging("/tmp/rr");
  RunWithString("RRR");
  double estimated_heading = std::fmod(
          (*status_queue_->ReadLastMessage())->estimated_heading() + M_PI * 2, M_PI * 2);
  EXPECT_NEAR(estimated_heading, 0.3, 0.01);
  EXPECT_NEAR((*status_queue_->ReadLastMessage())->estimated_x_position(), -6.95, 0.01);
  EXPECT_NEAR((*status_queue_->ReadLastMessage())->estimated_y_position(), -1.3, 0.01);
}

TEST_F(C2018DrivetrainTest, DISABLED_AutoRL) {
  BeginLogging("/tmp/rl");
  RunWithString("RLR");
  // Too much work to caulculate the ending position without any DrivePaths
  /*EXPECT_NEAR((*status_queue_->ReadLastMessage())->estimated_heading(), 0.0, 0.01);
  EXPECT_NEAR((*status_queue_->ReadLastMessage())->estimated_x_position(), 0, 0.01);
  EXPECT_NEAR((*status_queue_->ReadLastMessage())->estimated_y_position(), 0, 0.01);*/
}

TEST_F(C2018DrivetrainTest, DISABLED_AutoLL) {
  BeginLogging("/tmp/ll");
  RunWithString("LLR");
  // Too much work to caulculate the ending position without any DrivePaths
  /*EXPECT_NEAR((*status_queue_->ReadLastMessage())->estimated_heading(), 0.0, 0.01);
  EXPECT_NEAR((*status_queue_->ReadLastMessage())->estimated_x_position(), 0, 0.01);
  EXPECT_NEAR((*status_queue_->ReadLastMessage())->estimated_y_position(), 0, 0.01);*/
}

TEST_F(C2018DrivetrainTest, DISABLED_AutoLR) {
  BeginLogging("/tmp/lr");
  RunWithString("LRR");
  // Too much work to caulculate the ending position without any DrivePaths
  /*EXPECT_NEAR((*status_queue_->ReadLastMessage())->estimated_heading(), 0.0, 0.01);
  EXPECT_NEAR((*status_queue_->ReadLastMessage())->estimated_x_position(), 0, 0.01);
  EXPECT_NEAR((*status_queue_->ReadLastMessage())->estimated_y_position(), 0, 0.01);*/
}
