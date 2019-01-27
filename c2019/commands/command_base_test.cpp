#include <thread>

#include "c2019/commands/command_base.h"
#include "c2019/commands/test_auto.h"
#include "gtest/gtest.h"
#include "muan/queues/queue_manager.h"

namespace c2019 {
namespace commands {

using DrivetrainGoal = muan::subsystems::drivetrain::GoalProto;
using DrivetrainStatus = muan::subsystems::drivetrain::StatusProto;
using muan::queues::QueueManager;
using muan::wpilib::DriverStationProto;

TEST(C2019AutonomousTest, PathDriveTransformsZeroInit) {
  DriverStationProto ds_proto;
  ds_proto->set_is_sys_active(true);
  QueueManager<DriverStationProto>::Fetch()->WriteMessage(ds_proto);

  CommandBase auto_base;
  auto_base.SetFieldPosition(0.0, -4.0, -M_PI);
  auto_base.StartDrivePath(1.0, -3.0, -M_PI, -1);
  auto_base.ExitAutonomous();

  DrivetrainGoal goal;
  ASSERT_TRUE(QueueManager<DrivetrainGoal>::Fetch()->ReadLastMessage(&goal));
  ASSERT_TRUE(goal->has_path_goal());
  EXPECT_NEAR(goal->path_goal().x(), -1.0, 1e-3);
  EXPECT_NEAR(goal->path_goal().y(), -1.0, 1e-3);
  EXPECT_NEAR(goal->path_goal().heading(), 0.0, 1e-3);
  EXPECT_TRUE(goal->path_goal().backwards());
}

TEST(C2019AutonomousTest, PathDriveTransformsNonzeroInit) {
  DriverStationProto ds_proto;
  ds_proto->set_is_sys_active(true);
  QueueManager<DriverStationProto>::Fetch()->WriteMessage(ds_proto);

  CommandBase auto_base;
  {
    DrivetrainStatus status;
    status->set_estimated_x_position(1.0);
    status->set_estimated_y_position(-1.0);
    status->set_estimated_heading(-M_PI / 2);
    QueueManager<DrivetrainStatus>::Fetch()->WriteMessage(status);
  }
  auto_base.SetFieldPosition(0.0, -4.0, -M_PI);
  auto_base.StartDrivePath(1.0, -3.0, -M_PI, -1);

  DrivetrainGoal goal;
  ASSERT_TRUE(QueueManager<DrivetrainGoal>::Fetch()->ReadLastMessage(&goal));
  ASSERT_TRUE(goal->has_path_goal());
  EXPECT_NEAR(goal->path_goal().x(), 0.0, 1e-3);
  EXPECT_NEAR(goal->path_goal().y(), 0.0, 1e-3);
  EXPECT_NEAR(goal->path_goal().heading(), -M_PI / 2, 1e-3);
  EXPECT_TRUE(goal->path_goal().backwards());
}

void StartAutoInThread() {
  TestAuto command;
  std::thread command_thread(command);
  command_thread.detach();
}

TEST(C2019AutonomousTest, ThreadKill) {
  aos::time::PhasedLoop loop_{std::chrono::milliseconds(10)};
  AutoStatusProto auto_status;
  AutoGoalProto auto_goal;
  auto_goal->set_run_command(true);
  auto_goal->set_command(TEST_AUTO);
  DriverStationProto ds_proto;
  ds_proto->set_is_sys_active(true);
  QueueManager<AutoGoalProto>::Fetch()->WriteMessage(auto_goal);
  QueueManager<DriverStationProto>::Fetch()->WriteMessage(ds_proto);
  StartAutoInThread();
  for (int i = 0; i < 51; i++) {
    auto_goal->set_run_command(true);
    auto_goal->set_command(TEST_AUTO);
    ds_proto->set_is_sys_active(true);
    QueueManager<AutoGoalProto>::Fetch()->WriteMessage(auto_goal);
    QueueManager<DriverStationProto>::Fetch()->WriteMessage(ds_proto);
    if (i >= 1) {
      EXPECT_TRUE(QueueManager<AutoStatusProto>::Fetch()->ReadLastMessage(
          &auto_status));
      EXPECT_TRUE(auto_status->running_command());
    }
    loop_.SleepUntilNext();
  }
  loop_.SleepUntilNext();
  EXPECT_TRUE(
      QueueManager<AutoStatusProto>::Fetch()->ReadLastMessage(&auto_status));
  EXPECT_FALSE(auto_status->running_command());
}

TEST(C2019AutonomousTest, PullOut) {
  aos::time::PhasedLoop loop_{std::chrono::milliseconds(10)};
  AutoStatusProto auto_status;
  AutoGoalProto auto_goal;
  auto_goal->set_run_command(true);
  auto_goal->set_command(TEST_AUTO);
  DriverStationProto ds_proto;
  ds_proto->set_is_sys_active(true);
  QueueManager<AutoGoalProto>::Fetch()->WriteMessage(auto_goal);
  QueueManager<DriverStationProto>::Fetch()->WriteMessage(ds_proto);
  StartAutoInThread();
  for (int i = 0; i < 50; i++) {
    if (i < 30) {
      auto_goal->set_run_command(true);
      auto_goal->set_command(TEST_AUTO);
      ds_proto->set_is_sys_active(true);
    } else {
      auto_goal->set_run_command(true);
      auto_goal->set_command(
          DRIVE_STRAIGHT);  // Started running a different command
      ds_proto->set_is_sys_active(true);
    }

    QueueManager<AutoGoalProto>::Fetch()->WriteMessage(auto_goal);
    QueueManager<DriverStationProto>::Fetch()->WriteMessage(ds_proto);
    if (i >= 1 && i < 31) {
      EXPECT_TRUE(QueueManager<AutoStatusProto>::Fetch()->ReadLastMessage(
          &auto_status));
      EXPECT_TRUE(auto_status->running_command());
    } else {
      EXPECT_TRUE(QueueManager<AutoStatusProto>::Fetch()->ReadLastMessage(
          &auto_status));
      EXPECT_FALSE(auto_status->running_command());
    }
    loop_.SleepUntilNext();
  }
  EXPECT_TRUE(
      QueueManager<AutoStatusProto>::Fetch()->ReadLastMessage(&auto_status));
  EXPECT_FALSE(auto_status->running_command());
}

}  // namespace commands
}  // namespace c2019
