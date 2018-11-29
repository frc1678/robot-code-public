#ifndef O2018_AUTONOMOUS_AUTONOMOUS_BASE_H_
#define O2018_AUTONOMOUS_AUTONOMOUS_BASE_H_

#include <string>

#include "Eigen/Dense"
#include "gtest/gtest.h"
#include "muan/subsystems/drivetrain/queue_types.h"
#include "muan/webdash/queue_types.h"
#include "muan/wpilib/queue_types.h"
#include "o2018/subsystems/arm/queue_types.h"
#include "third_party/aos/common/util/phased_loop.h"

namespace o2018 {
namespace autonomous {

using DrivetrainGoal = muan::subsystems::drivetrain::GoalProto;
using DrivetrainStatus = muan::subsystems::drivetrain::StatusProto;
using ArmGoalProto = o2018::subsystems::arm::ArmGoalProto;
using IntakeMode = o2018::subsystems::arm::IntakeMode;

class AutonomousBase {
 public:
  AutonomousBase();

 protected:
  FRIEND_TEST(O2018AutonomousTest, PathDriveTransformsZeroInit);
  FRIEND_TEST(O2018AutonomousTest, PathDriveTransformsNonzeroInit);
  bool IsAutonomous();

  void Wait(uint32_t num_cycles);
  void FreeArm();

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
  void SetArm(double angle, IntakeMode intake_mode);

  // Set the robot-space (robot poweron position) transformation. The parameters
  // are the position of the robot (right now) in field coordinates (F).

  muan::wpilib::DriverStationQueue::QueueReader driver_station_reader_;
  muan::wpilib::GameSpecificStringQueue::QueueReader
      game_specific_string_reader_;
  muan::subsystems::drivetrain::GoalQueue* drivetrain_goal_queue_;
  muan::subsystems::drivetrain::StatusQueue::QueueReader
      drivetrain_status_reader_;

  o2018::subsystems::arm::ArmGoalQueue* arm_goal_queue_;

  Eigen::Transform<double, 2, Eigen::AffineCompact> transform_f0_;
  double theta_offset_ = 0.0;

  double max_path_acceleration_;
  double max_path_velocity_;

  aos::time::PhasedLoop loop_{std::chrono::milliseconds(10)};
};

}  // namespace autonomous
}  // namespace o2018

#endif  // O2018_AUTONOMOUS_AUTONOMOUS_BASE_H_
