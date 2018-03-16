#ifndef C2018_AUTONOMOUS_AUTONOMOUS_BASE_H_
#define C2018_AUTONOMOUS_AUTONOMOUS_BASE_H_

#include <string>

#include "c2018/subsystems/score_subsystem/queue_types.h"
#include "c2018/subsystems/score_subsystem/score_subsystem.pb.h"
#include "gtest/gtest.h"
#include "muan/webdash/queue_types.h"
#include "muan/wpilib/queue_types.h"
#include "third_party/aos/common/util/phased_loop.h"
#include "third_party/frc971/control_loops/drivetrain/drivetrain_config.h"
#include "third_party/frc971/control_loops/drivetrain/queue_types.h"

namespace c2018 {
namespace autonomous {

constexpr double kCubeX =
    5.18;  // Platform zone cubes' y-value if robot starts facing the wall
constexpr double kScaleX = 6.9;  // Scale y-value if robot starts facing wall
constexpr double kSwitchFrontX =
    2.9;  // FRONT of the switch if robot starts facing wall

class AutonomousBase {
 public:
  AutonomousBase();

 protected:
  FRIEND_TEST(C2018AutonomousTest, PathDriveTransformsZeroInit);
  FRIEND_TEST(C2018AutonomousTest, PathDriveTransformsNonzeroInit);

  bool IsAutonomous();

  void StartDriveAbsolute(double left, double right,
                          bool follow_through = false);
  void StartDriveRelative(double forward, double theta,
                          double final_velocity = 0.0);
  // Direction: 1 => forwards, 0 => autodetect, -1 => backwards
  // x, y, and heading are given in _field_ space. x is forwards, away from the
  // driver station wall. y is to the left looking out of the driver station
  // wall. heading is counterclockwise positive, with 0 being headed straight in
  // the x direction.
  void StartDrivePath(double x, double y, double heading,
                      int force_direction = 0,
                      frc971::control_loops::drivetrain::Gear gear =
                          frc971::control_loops::drivetrain::Gear::kHighGear,
                      double extra_distance_initial = 0,
                      double extra_distance_final = 0);
  void StartDriveAtAngle(double distance, double theta_absolute,
                         double final_velocity = 0.0);

  bool IsDriveComplete();
  bool IsDrivetrainNear(double x, double y, double distance);
  void WaitUntilDriveComplete();
  void WaitUntilDrivetrainNear(double x, double y, double distance);
  void WaitUntilElevatorAtPosition();

  void Wait(uint32_t num_cycles);

  void IntakeGround();
  void IntakeOpen();
  void IntakeClose();
  void StopIntakeGround();
  void GoToIntake();

  void MoveToSwitch();
  void MoveToScale(bool front);
  void MoveTo(c2018::score_subsystem::ScoreGoal goal);
  void Score(bool fast = true);
  void StopScore();
  bool IsAtScoreHeight();
  bool HasCube();
  void WaitForCube();

  // Set the robot-space (robot poweron position) transformation. The parameters
  // are the position of the robot (right now) in field coordinates (F).
  void SetFieldPosition(double x, double y, double theta);

  double max_forward_velocity_ = 3.0, max_forward_acceleration_ = 3.0;
  double max_angular_velocity_ = 5.0, max_angular_acceleration_ = 4.0;
  double max_path_acceleration_ = 3.0;

  // Follow through storage
  bool follow_through_ = false;
  double goal_dist_;
  // Are we crossing in the positive direction?
  bool threshold_positive_ = true;

  frc971::control_loops::drivetrain::DrivetrainConfig config_;
  frc971::control_loops::drivetrain::GoalQueue* drivetrain_goal_queue_;
  frc971::control_loops::drivetrain::StatusQueue::QueueReader
      drivetrain_status_reader_;
  c2018::score_subsystem::ScoreSubsystemGoalQueue* score_goal_queue_;
  c2018::score_subsystem::ScoreSubsystemStatusQueue::QueueReader
      score_status_reader_;
  muan::wpilib::DriverStationQueue::QueueReader driver_station_reader_;
  muan::wpilib::GameSpecificStringQueue::QueueReader
      game_specific_string_reader_;

  aos::time::PhasedLoop loop_{std::chrono::milliseconds(5)};

  // The position and orientation of the field's origin relative to the robot's
  // origin (at time of power-on. i.e. "robot to field").
  Eigen::Transform<double, 2, Eigen::AffineCompact> transform_f0_;
  double theta_offset_ = 0.0;
};

}  // namespace autonomous
}  // namespace c2018

#endif  // C2018_AUTONOMOUS_AUTONOMOUS_BASE_H_
