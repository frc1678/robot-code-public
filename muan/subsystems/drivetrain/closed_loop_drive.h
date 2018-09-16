#ifndef MUAN_SUBSYSTEMS_DRIVETRAIN_CLOSED_LOOP_DRIVE_H_
#define MUAN_SUBSYSTEMS_DRIVETRAIN_CLOSED_LOOP_DRIVE_H_

#include "muan/control/trajectory.h"
#include "muan/control/nonlinear_feedback_controller.h"
#include "muan/subsystems/drivetrain/queue_types.h"
#include "muan/subsystems/drivetrain/drivetrain_config.h"

namespace muan {
namespace subsystems {
namespace drivetrain {

using muan::control::DrivetrainModel;
using muan::control::DriveTransmission;
using muan::control::NonLinearFeedbackController;
using muan::control::Trajectory;
using muan::control::Pose;

class ClosedLoopDrive {
 public:
  ClosedLoopDrive(DrivetrainConfig dt_config,
                  Eigen::Vector2d* cartesian_position,
                  double* integrated_heading,
                  Eigen::Vector2d* linear_angular_velocity);

  void SetGoal(const GoalProto& goal);
  void Update(OutputProto* output, StatusProto* status);

 private:
  DrivetrainModel model_;
  NonLinearFeedbackController controller_;
  DrivetrainConfig dt_config_;

  Eigen::Vector2d* cartesian_position_;
  double* integrated_heading_;
  Eigen::Vector2d* linear_angular_velocity_;

  Trajectory trajectory_;
  Pose last_goal_pose_;

  bool high_gear_;
};

}  // namespace drivetrain
}  // namespace subsystems
}  // namespace muan

#endif  // MUAN_SUBSYSTEMS_DRIVETRAIN_CLOSED_LOOP_DRIVE_H_
