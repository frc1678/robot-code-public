#ifndef C2019_COMMANDS_COMMAND_BASE_H_
#define C2019_COMMANDS_COMMAND_BASE_H_

#include <string>

#include "Eigen/Dense"
#include "c2019/commands/queue_types.h"
#include "c2019/subsystems/superstructure/queue_types.h"
#include "gtest/gtest.h"
#include "muan/subsystems/drivetrain/queue_types.h"
#include "muan/webdash/queue_types.h"
#include "muan/wpilib/queue_types.h"
#include "third_party/aos/common/util/phased_loop.h"

namespace c2019 {
namespace commands {

using DrivetrainGoal = muan::subsystems::drivetrain::GoalProto;
using DrivetrainStatus = muan::subsystems::drivetrain::StatusProto;
using DrivetrainInput = muan::subsystems::drivetrain::InputProto;

class CommandBase {
 public:
  CommandBase();

 protected:
  FRIEND_TEST(C2019AutonomousTest, PathDriveTransformsZeroInit);
  FRIEND_TEST(C2019AutonomousTest, PathDriveTransformsNonzeroInit);
  bool IsAutonomous();
  void EnterAutonomous();
  void ExitAutonomous();
  void Wait(uint32_t num_cycles);

  void StartDrivePath(double x, double y, double heading,
                      int force_direction = 0, bool gear = true,
                      bool full_send = false, double extra_distance_initial = 0,
                      double extra_distance_final = 0,
                      double path_voltage = 12.0);
  void StartPointTurn(double theta);

  bool StartDriveVision(double target_dist = 0.73);
  bool StartDriveVisionBottom();
  bool StartDriveVisionBackwards();
  void HoldPosition();

  bool IsDriveComplete();
  bool IsDrivetrainNear(double x, double y, double distance);

  void WaitUntilDriveComplete();
  void WaitUntilDrivetrainNear(double x, double y, double distance);

  void SetFieldPosition(double x, double y, double theta);

  void GoTo(
      superstructure::ScoreGoal score_goal,
      superstructure::IntakeGoal intake_goal = superstructure::INTAKE_NONE);

  void ScoreHatch(int num_ticks);

  void WaitForElevatorAndLL();
  void WaitForHatch();

  // Set the robot-space (robot poweron position) transformation. The parameters
  // are the position of the robot (right now) in field coordinates (F).

  muan::wpilib::DriverStationQueue::QueueReader driver_station_reader_;
  muan::wpilib::GameSpecificStringQueue::QueueReader
      game_specific_string_reader_;
  muan::subsystems::drivetrain::GoalQueue* drivetrain_goal_queue_;
  muan::subsystems::drivetrain::StatusQueue::QueueReader
      drivetrain_status_reader_;
  c2019::commands::AutoStatusQueue* auto_status_queue_;
  c2019::commands::AutoGoalQueue::QueueReader auto_goal_reader_;
  Eigen::Transform<double, 2, Eigen::AffineCompact> transform_f0_;
  double theta_offset_ = 0.0;

  double max_lin_ = 7.0;
  double max_acc_ = 9.5;

  aos::time::PhasedLoop loop_{std::chrono::milliseconds(10)};
};

}  // namespace commands
}  // namespace c2019

#endif  // C2019_COMMANDS_COMMAND_BASE_H_
