#ifndef MUAN_SUBSYSTEMS_DRIVETRAIN_CLOSED_LOOP_DRIVE_H_
#define MUAN_SUBSYSTEMS_DRIVETRAIN_CLOSED_LOOP_DRIVE_H_

#include "muan/control/nonlinear_feedback_controller.h"
#include "muan/control/trajectory.h"
#include "muan/subsystems/drivetrain/drivetrain_config.h"
#include "muan/subsystems/drivetrain/queue_types.h"

namespace muan {
namespace subsystems {
namespace drivetrain {

using muan::control::DrivetrainModel;
using muan::control::DriveTransmission;
using muan::control::NonLinearFeedbackController;
using muan::control::Pose;
using muan::control::Trajectory;

class ClosedLoopDrive {
 public:
  ClosedLoopDrive(DrivetrainConfig dt_config,
                  Eigen::Vector2d* cartesian_position,
                  double* integrated_heading,
                  Eigen::Vector2d* linear_angular_velocity,
                  Eigen::Vector2d* left_right_position);

  void SetGoal(const GoalProto& goal);
  void Update(OutputProto* output, StatusProto* status);

 private:
  void UpdatePointTurn(OutputProto* output, StatusProto* status);
  void UpdateVisionArc(OutputProto* output, StatusProto* status);
  void UpdateDistance(OutputProto* output, StatusProto* status);
  void UpdateLeftRightManual(OutputProto* output, StatusProto* status);
  void UpdatePathFollower(OutputProto* output, StatusProto* status);
  void UpdateLinearAngularVelocity(OutputProto* output);

  DrivetrainModel model_;
  NonLinearFeedbackController controller_;
  DrivetrainConfig dt_config_;

  Eigen::Vector2d* cartesian_position_;
  double* integrated_heading_;
  Eigen::Vector2d* linear_angular_velocity_;
  Eigen::Vector2d* left_right_position_;

  Trajectory trajectory_;
  Pose last_goal_pose_;

  bool high_gear_;

  double point_turn_goal_;
  double distance_goal_;
  double prev_heading_;
  double left_pos_goal_;
  double right_pos_goal_;
  Eigen::Vector2d prev_left_right_;
  double lin_vel_goal_;
  double ang_vel_goal_;
  ControlMode control_mode_;
};

}  // namespace drivetrain
}  // namespace subsystems
}  // namespace muan

#endif  // MUAN_SUBSYSTEMS_DRIVETRAIN_CLOSED_LOOP_DRIVE_H_
