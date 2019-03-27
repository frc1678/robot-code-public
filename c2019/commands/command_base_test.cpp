#include <thread>

#include "c2019/commands/command_base.h"
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
  ds_proto->set_mode(RobotMode::AUTONOMOUS);
  QueueManager<DriverStationProto>::Fetch()->WriteMessage(ds_proto);

  CommandBase auto_base;
  auto_base.SetFieldPosition(0.0, -4.0, -M_PI);
  auto_base.StartDrivePath(1.0, -3.0, -M_PI, -1);

  DrivetrainGoal goal;
  QueueManager<DrivetrainGoal>::Fetch()->ReadLastMessage(&goal);
  ASSERT_TRUE(goal->has_path_goal());
  EXPECT_NEAR(goal->path_goal().x(), -1.0, 1e-3);
  EXPECT_NEAR(goal->path_goal().y(), -1.0, 1e-3);
  EXPECT_NEAR(goal->path_goal().heading(), 0.0, 1e-3);
  EXPECT_TRUE(goal->path_goal().backwards());
}

TEST(C2019AutonomousTest, PathDriveTransformsNonzeroInit) {
  DriverStationProto ds_proto;
  ds_proto->set_is_sys_active(true);
  ds_proto->set_mode(RobotMode::AUTONOMOUS);
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
  QueueManager<DrivetrainGoal>::Fetch()->ReadLastMessage(&goal);
  ASSERT_TRUE(goal->has_path_goal());
  EXPECT_NEAR(goal->path_goal().x(), 0.0, 1e-3);
  EXPECT_NEAR(goal->path_goal().y(), 0.0, 1e-3);
  EXPECT_NEAR(goal->path_goal().heading(), -M_PI / 2, 1e-3);
  EXPECT_TRUE(goal->path_goal().backwards());
}


}  // namespace commands
}  // namespace c2019
