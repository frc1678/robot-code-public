#ifndef C2019_AUTONOMOUS_AUTONOMOUS_BASE_H_
#define C2019_AUTONOMOUS_AUTONOMOUS_BASE_H_

#include <string>

#include "Eigen/Dense"
#include "gtest/gtest.h"
#include "muan/subsystems/drivetrain/queue_types.h"
#include "muan/webdash/queue_types.h"
#include "muan/wpilib/queue_types.h"
#include "third_party/aos/common/util/phased_loop.h"
#include "c2019/autonomous/queue_types.h"

namespace c2019 {
namespace autonomous {

using DrivetrainGoal = muan::subsystems::drivetrain::GoalProto;
using DrivetrainStatus = muan::subsystems::drivetrain::StatusProto;

class AutonomousBase {
 public:
  AutonomousBase();

 protected:
  FRIEND_TEST(C2019AutonomousTest, PathDriveTransformsZeroInit);
  FRIEND_TEST(C2019AutonomousTest, PathDriveTransformsNonzeroInit);
  bool IsAutonomous();
  void ExitAutonomous();

  void Wait(uint32_t num_cycles);

  void StartDrivePath(double x, double y, double heading,
                      int force_direction = 0, bool gear = true,
                      double extra_distance_initial = 0,
                      double extra_distance_final = 0,
                      double path_voltage = 9.0);

  bool IsDriveComplete();
  bool IsDrivetrainNear(double x, double y, double distance);

  void WaitUntilDriveComplete();
  void WaitUntilDrivetrainNear(double x, double y, double distance);

  void SetFieldPosition(double x, double y, double theta);

  // Set the robot-space (robot poweron position) transformation. The parameters
  // are the position of the robot (right now) in field coordinates (F).

  muan::wpilib::DriverStationQueue::QueueReader driver_station_reader_;
  muan::wpilib::GameSpecificStringQueue::QueueReader
      game_specific_string_reader_;
  muan::subsystems::drivetrain::GoalQueue* drivetrain_goal_queue_;
  muan::subsystems::drivetrain::StatusQueue::QueueReader
      drivetrain_status_reader_;
  c2019::autonomous::AutoStatusQueue* auto_status_queue_;

  Eigen::Transform<double, 2, Eigen::AffineCompact> transform_f0_;
  double theta_offset_ = 0.0;

  double max_path_acceleration_;
  double max_path_velocity_;

  aos::time::PhasedLoop loop_{std::chrono::milliseconds(10)};
};

}  // namespace autonomous
}  // namespace c2019

#endif  // C2019_AUTONOMOUS_AUTONOMOUS_BASE_H_
