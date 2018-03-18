#include "c2018/autonomous/autonomous_base.h"
#include "gtest/gtest.h"
#include "muan/queues/queue_manager.h"

namespace c2018 {
namespace autonomous {

using DrivetrainGoal = frc971::control_loops::drivetrain::GoalProto;
using DrivetrainStatus = frc971::control_loops::drivetrain::StatusProto;
using muan::queues::QueueManager;

TEST(C2018AutonomousTest, PathDriveTransformsZeroInit) {
  AutonomousBase auto_base;
  auto_base.SetFieldPosition(0.0, -4.0, -M_PI);
  auto_base.StartDrivePath(1.0, -3.0, -M_PI, -1);

  DrivetrainGoal goal;
  ASSERT_TRUE(QueueManager<DrivetrainGoal>::Fetch()->ReadLastMessage(&goal));
  ASSERT_TRUE(goal->has_path_command());
  EXPECT_NEAR(goal->path_command().x_goal(), -1.0, 1e-3);
  EXPECT_NEAR(goal->path_command().y_goal(), -1.0, 1e-3);
  EXPECT_NEAR(goal->path_command().theta_goal(), 0.0, 1e-3);
  EXPECT_TRUE(goal->path_command().backwards());
}

TEST(C2018AutonomousTest, PathDriveTransformsNonzeroInit) {
  AutonomousBase auto_base;
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
  ASSERT_TRUE(goal->has_path_command());
  EXPECT_NEAR(goal->path_command().x_goal(), 0.0, 1e-3);
  EXPECT_NEAR(goal->path_command().y_goal(), 0.0, 1e-3);
  EXPECT_NEAR(goal->path_command().theta_goal(), -M_PI / 2, 1e-3);
  EXPECT_TRUE(goal->path_command().backwards());
}

}  // namespace autonomous
}  // namespace c2018
