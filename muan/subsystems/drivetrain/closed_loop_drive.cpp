#include "muan/subsystems/drivetrain/closed_loop_drive.h"
#include <limits>

namespace muan {
namespace subsystems {
namespace drivetrain {

using muan::control::HermiteSpline;

ClosedLoopDrive::ClosedLoopDrive(DrivetrainConfig dt_config,
                                 Eigen::Vector2d* cartesian_position,
                                 double* integrated_heading,
                                 Eigen::Vector2d* linear_angular_velocity,
                                 Eigen::Vector2d* left_right_position)
    : model_{dt_config.drive_properties,
             DriveTransmission(dt_config.low_gear_properties),
             DriveTransmission(dt_config.high_gear_properties)},
      controller_{model_, dt_config.beta, dt_config.zeta},
      dt_config_{dt_config},
      cartesian_position_{cartesian_position},
      integrated_heading_{integrated_heading},
      linear_angular_velocity_{linear_angular_velocity},
      left_right_position_{left_right_position} {}

void ClosedLoopDrive::SetGoal(const GoalProto& goal) {
  if (goal->has_point_turn_goal()) {
    if (goal->point_turn_goal() != point_turn_goal_) {
      point_turn_goal_ = goal->point_turn_goal();
      prev_heading_ = *integrated_heading_;
      prev_left_right_ = *left_right_position_;
      distance_goal_ = 0;
    }
    control_mode_ = ControlMode::POINT_TURN;
    high_gear_ = false;
    return;
  }

  if (goal->has_arc_goal()) {
    point_turn_goal_ = goal->arc_goal().angular();
    prev_heading_ = *integrated_heading_;
    prev_left_right_ = *left_right_position_;
    distance_goal_ = goal->arc_goal().linear();
    control_mode_ = ControlMode::VISION_ARC;
    return;
  }

  if (goal->has_distance_goal()) {
    if (goal->distance_goal() != distance_goal_) {
      distance_goal_ = goal->distance_goal();
      prev_heading_ = *integrated_heading_;
      prev_left_right_ = *left_right_position_;
    }
    control_mode_ = ControlMode::DISTANCE;
    high_gear_ = false;
    return;
  }

  if (goal->has_left_right_goal()) {
    left_pos_goal_ = goal->left_right_goal().left_goal();
    right_pos_goal_ = goal->left_right_goal().right_goal();

    control_mode_ = ControlMode::LEFT_RIGHT;
    high_gear_ = false;
    return;
  }

  if (goal->has_linear_angular_velocity_goal()) {
    lin_vel_goal_ = goal->linear_angular_velocity_goal().linear_velocity();
    ang_vel_goal_ = goal->linear_angular_velocity_goal().angular_velocity();
    control_mode_ = ControlMode::LINEAR_ANGULAR_VEL;
    return;
  }

  if (goal->has_path_goal()) {
    control_mode_ = ControlMode::PATH_FOLLOWING;
    const auto path_goal = goal->path_goal();
    const Pose goal_pose{
        Eigen::Vector3d(path_goal.x(), path_goal.y(), path_goal.heading())};

    if (goal_pose.Get() == last_goal_pose_.Get()) {
      return;
    } else {
      last_goal_pose_ = goal_pose;
    }

    const Pose inital_pose{*cartesian_position_, *integrated_heading_};

    Eigen::Vector2d linear_angular_velocity = *linear_angular_velocity_;
    if (path_goal.full_send()) {
      linear_angular_velocity = Eigen::Vector2d(0, 0);
    }
    const HermiteSpline path{inital_pose,
                             goal_pose,
                             (linear_angular_velocity)(0),
                             path_goal.final_velocity(),
                             path_goal.backwards(),
                             path_goal.extra_distance_initial(),
                             path_goal.extra_distance_final(),
                             (linear_angular_velocity)(1),
                             path_goal.final_angular_velocity()};

    const double max_velocity = path_goal.has_max_linear_velocity()
                                    ? path_goal.max_linear_velocity()
                                    : dt_config_.max_velocity;
    const double max_voltage = path_goal.max_voltage();
    const double max_acceleration = path_goal.has_max_linear_accel()
                                        ? path_goal.max_linear_accel()
                                        : dt_config_.max_acceleration;
    const double max_centripetal_acceleration =
        path_goal.has_max_centripetal_accel()
            ? path_goal.max_centripetal_accel()
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
}

void ClosedLoopDrive::Update(OutputProto* output, StatusProto* status) {
  if (control_mode_ == ControlMode::POINT_TURN) {
    UpdatePointTurn(output, status);
  } else if (control_mode_ == ControlMode::DISTANCE) {
    UpdateDistance(output, status);
  } else if (control_mode_ == ControlMode::LEFT_RIGHT) {
    UpdateLeftRightManual(output, status);
  } else if (control_mode_ == ControlMode::PATH_FOLLOWING) {
    UpdatePathFollower(output, status, (*status)->dt());
  } else if (control_mode_ == ControlMode::LINEAR_ANGULAR_VEL) {
    UpdateLinearAngularVelocity(output);
  } else if (control_mode_ == ControlMode::VISION_ARC) {
    UpdateVisionArc(output, status);
  }
}

void ClosedLoopDrive::UpdatePointTurn(OutputProto* output,
                                      StatusProto* status) {
  Eigen::Vector2d delta =
      model_.InverseKinematics(Eigen::Vector2d(0., point_turn_goal_));
  (*output)->set_output_type(POSITION);
  (*output)->set_left_setpoint(delta(0) + prev_left_right_(0));
  (*output)->set_right_setpoint(delta(1) + prev_left_right_(1));

  bool left_complete = std::abs((*left_right_position_)(0) - (*output)->left_setpoint()) < .3;
  bool right_complete = std::abs((*left_right_position_)(1) - (*output)->right_setpoint()) < .3;
  (*status)->set_point_turn_complete(left_complete && right_complete && (*linear_angular_velocity_)(1) < 0.1);
  (*status)->set_heading_error((prev_heading_ + point_turn_goal_) -
                               *integrated_heading_);
}

void ClosedLoopDrive::UpdateVisionArc(OutputProto* output,
                                      StatusProto* status) {
  Eigen::Vector2d delta = model_.InverseKinematics(
      Eigen::Vector2d(distance_goal_, point_turn_goal_));

  (*status)->set_heading_error(point_turn_goal_ - *integrated_heading_);
  (*status)->set_closed_loop_control_mode(control_mode_);

  (*output)->set_output_type(ARC);
  (*output)->set_left_setpoint(delta(0) + prev_left_right_(0));
  (*output)->set_right_setpoint(delta(1) + prev_left_right_(1));
  (*output)->set_yaw(point_turn_goal_ -
                     ((*linear_angular_velocity_)(1) * 0.01));
  (*output)->set_arc_vel(distance_goal_);
}

void ClosedLoopDrive::UpdateDistance(OutputProto* output, StatusProto* status) {
  Eigen::Vector2d delta =
      model_.InverseKinematics(Eigen::Vector2d(distance_goal_, 0));

  (*status)->set_closed_loop_control_mode(control_mode_);

  (*output)->set_output_type(POSITION);
  (*output)->set_left_setpoint(delta(0) + prev_left_right_(0));
  (*output)->set_right_setpoint(delta(1) + prev_left_right_(1));
}

void ClosedLoopDrive::UpdateLeftRightManual(OutputProto* output,
                                            StatusProto* status) {
  (*status)->set_closed_loop_control_mode(control_mode_);

  (*output)->set_output_type(POSITION);
  (*output)->set_left_setpoint(left_pos_goal_);
  (*output)->set_right_setpoint(right_pos_goal_);
}

void ClosedLoopDrive::UpdateLinearAngularVelocity(OutputProto* output) {
  auto left_right =
      model_.InverseKinematics(Eigen::Vector2d(lin_vel_goal_, ang_vel_goal_));
  (*output)->set_output_type(VELOCITY);
  (*output)->set_left_setpoint(left_right(0));
  (*output)->set_right_setpoint(left_right(1));
}

void ClosedLoopDrive::UpdatePathFollower(OutputProto* output,
                                         StatusProto* status, double dt) {
  const Pose current{*cartesian_position_, *integrated_heading_};
  const Trajectory::TimedPose goal = trajectory_.Advance(dt);
  const Pose error = goal.pose.pose() - current;

  Eigen::Vector2d goal_velocity;
  goal_velocity(0) = goal.v;
  goal_velocity(1) = goal.pose.curvature() * goal.v;

  Eigen::Vector2d goal_accel;
  goal_accel(0) = goal.a;
  goal_accel(1) = goal.pose.curvature() * goal.a;

  auto setpoint =
      controller_.Update(goal_velocity, goal_accel, current, error, high_gear_);

  (*status)->set_x_error(error.Get()(0));
  (*status)->set_y_error(error.Get()(1));
  (*status)->set_heading_error(error.Get()(2));

  (*status)->set_x_goal(last_goal_pose_.Get()(0));
  (*status)->set_y_goal(last_goal_pose_.Get()(1));
  (*status)->set_heading_goal(last_goal_pose_.Get()(2));

  (*status)->set_profiled_x_goal(goal.pose.Get()(0));
  (*status)->set_profiled_y_goal(goal.pose.Get()(1));
  (*status)->set_profiled_heading_goal(goal.pose.Get()(2));
  (*status)->set_profiled_velocity_goal(goal.v);
  (*status)->set_profiled_acceleration_goal(goal.a);
  (*status)->set_profiled_curvature_goal(goal.pose.curvature());
  (*status)->set_adjusted_velocity_goal(
      model_.ForwardKinematics(setpoint.velocity)(0));
  (*status)->set_profile_complete(trajectory_.done());

  (*output)->set_output_type(VELOCITY);
  (*output)->set_left_setpoint(setpoint.velocity(0));
  (*output)->set_right_setpoint(setpoint.velocity(1));
  (*output)->set_left_setpoint_ff(
      trajectory_.done() ? 0 : setpoint.feedforwards(0));
  (*output)->set_right_setpoint_ff(
      trajectory_.done() ? 0 : setpoint.feedforwards(1));
}

}  // namespace drivetrain
}  // namespace subsystems
}  // namespace muan
