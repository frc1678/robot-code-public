#include "muan/subsystems/drivetrain/closed_loop_drive.h"
#include <limits>

namespace muan {
namespace subsystems {
namespace drivetrain {

using muan::control::HermiteSpline;

ClosedLoopDrive::ClosedLoopDrive(DrivetrainConfig dt_config,
                                 Eigen::Vector2d* cartesian_position,
                                 double* integrated_heading,
                                 Eigen::Vector2d* linear_angular_velocity)
    : model_{dt_config.drive_properties,
             DriveTransmission(dt_config.low_gear_properties),
             DriveTransmission(dt_config.high_gear_properties)},
      controller_{model_, dt_config.beta, dt_config.zeta},
      dt_config_{dt_config},
      cartesian_position_{cartesian_position},
      integrated_heading_{integrated_heading},
      linear_angular_velocity_{linear_angular_velocity} {}

void ClosedLoopDrive::SetGoal(const GoalProto& goal) {
  const auto path_goal = goal->path_goal();
  const Pose goal_pose{
      Eigen::Vector3d(path_goal.x(), path_goal.y(), path_goal.heading())};

  if (goal_pose.Get() == last_goal_pose_.Get()) {
    return;
  } else {
    last_goal_pose_ = goal_pose;
  }

  const Pose inital_pose{*cartesian_position_, *integrated_heading_};

  const HermiteSpline path{inital_pose,
                           goal_pose,
                           (*linear_angular_velocity_)(0),
                           path_goal.final_velocity(),
                           path_goal.backwards(),
                           path_goal.extra_distance_initial(),
                           path_goal.extra_distance_final(),
                           (*linear_angular_velocity_)(1),
                           path_goal.final_angular_velocity()};

  const double max_velocity =
      path_goal.has_linear_constraints()
          ? path_goal.linear_constraints().max_velocity()
          : dt_config_.max_velocity;
  const double max_voltage = path_goal.max_voltage();
  const double max_acceleration =
      path_goal.has_linear_constraints()
          ? path_goal.linear_constraints().max_acceleration()
          : dt_config_.max_acceleration;
  const double max_centripetal_acceleration =
      path_goal.has_angular_constraints()
          ? path_goal.angular_constraints().max_acceleration()
          : dt_config_.max_centripetal_acceleration;

  const double initial_velocity = (*linear_angular_velocity_)(0);
  const double final_velocity = path_goal.final_velocity();

  const Trajectory::Constraints constraints{
      max_velocity,                  //  NOLINT
      max_voltage,                   //  NOLINT
      max_acceleration,              //  NOLINT
      max_centripetal_acceleration,  //  NOLINT
      initial_velocity,              //  NOLINT
      final_velocity,                //  NOLINT
  };

  trajectory_ = Trajectory(path, constraints, goal->high_gear(), model_);
  high_gear_ = goal->high_gear();
}

void ClosedLoopDrive::Update(OutputProto* output, StatusProto* status) {
  const Pose current{*cartesian_position_, *integrated_heading_};
  const Trajectory::TimedPose goal = trajectory_.Advance(dt_config_.dt);
  const Pose error = goal.pose.pose() - current;

  Eigen::Vector2d goal_velocity;
  goal_velocity(0) = goal.v;
  goal_velocity(1) = goal.pose.curvature() * goal.v;

  auto setpoint = controller_.Update(goal_velocity, current, error, high_gear_);

  (*status)->set_x_error(error.Get()(0));
  (*status)->set_y_error(error.Get()(1));
  (*status)->set_heading_error(error.Get()(2));

  (*status)->set_profiled_x_goal(goal.pose.Get()(0));
  (*status)->set_profiled_y_goal(goal.pose.Get()(1));
  (*status)->set_profiled_heading_goal(goal.pose.Get()(2));
  (*status)->set_profiled_velocity_goal(goal.v);
  (*status)->set_adjusted_velocity_goal(
      model_.ForwardKinematics(setpoint.velocity)(0));

  (*output)->set_output_type(VELOCITY);
  (*output)->set_left_setpoint(setpoint.velocity(0));
  (*output)->set_right_setpoint(setpoint.velocity(1));
  (*output)->set_left_setpoint_ff(setpoint.feedforwards(0));
  (*output)->set_right_setpoint_ff(setpoint.feedforwards(1));
}

}  // namespace drivetrain
}  // namespace subsystems
}  // namespace muan
