#include "muan/control/trajectory.h"

namespace muan {
namespace control {

Trajectory::Trajectory(const HermiteSpline& spline, Constraints constraints,
                       bool high_gear, const DrivetrainModel& model) {
  TimeReparametrize(spline, model, high_gear, constraints);
}

void Trajectory::TimeReparametrize(const HermiteSpline& spline,
                                   const DrivetrainModel& model, bool high_gear,
                                   Constraints constraints) {
  std::array<PoseWithCurvature, kNumSamples> poses = spline.Populate(0., 1.);
  std::array<ConstrainedPose, kNumSamples> constrained_poses;

  bool backwards = spline.backwards();

  ConstrainedPose predecessor{
      .pose = poses.front(),
      .distance = 0.,
      .max_velocity = constraints.initial_velocity,
      .min_acceleration = -constraints.max_acceleration,
      .max_acceleration = constraints.max_acceleration,
  };

  constrained_poses.at(0) = predecessor;

  // Forward pass
  for (int i = 0; i < kNumSamples; i++) {
    // Convenience
    ConstrainedPose& constrained_pose = constrained_poses.at(i);

    // Begin constraining based on predecessor
    constrained_pose.pose = poses.at(i);
    double ds = (constrained_pose.pose.translational() -
                 predecessor.pose.translational())
                    .norm();
    constrained_pose.distance = ds + predecessor.distance;

    constrained_pose.max_velocity =
        std::min(constraints.max_velocity,
                 std::sqrt(predecessor.max_velocity * predecessor.max_velocity +
                           2. * predecessor.max_acceleration * ds));

    constrained_pose.min_acceleration = -constraints.max_acceleration;
    constrained_pose.max_acceleration = constraints.max_acceleration;

    constrained_pose.max_velocity =
        std::min(constrained_pose.max_velocity,
                 std::sqrt(std::abs(constraints.max_centripetal_acceleration /
                                    constrained_pose.pose.curvature())));

    Eigen::Vector2d linear_angular_velocity;
    linear_angular_velocity(0) =
        constrained_pose.max_velocity * (backwards ? -1. : 1.);
    linear_angular_velocity(1) = constrained_pose.max_velocity *
                                 constrained_pose.pose.curvature() *
                                 (backwards ? -1. : 1.);

    Bounds min_max_accel = model.CalculateMinMaxAcceleration(
        linear_angular_velocity, constrained_pose.pose.curvature(),
        constraints.max_voltage, high_gear);

    constrained_pose.min_acceleration =
        std::max(constrained_pose.min_acceleration,
                 backwards ? -min_max_accel.max : min_max_accel.min);

    constrained_pose.max_acceleration =
        std::min(constrained_pose.max_acceleration,
                 backwards ? -min_max_accel.min : min_max_accel.max);

    predecessor = constrained_pose;
  }

  ConstrainedPose successor{
      .pose = poses.front(),
      .distance = constrained_poses.back().distance,
      .max_velocity = constraints.final_velocity,
      .min_acceleration = -constraints.max_acceleration,
      .max_acceleration = constraints.max_acceleration,
  };

  // Backward pass
  for (int i = kNumSamples - 1; i >= 0; i--) {
    ConstrainedPose& constrained_pose = constrained_poses.at(i);
    double ds = constrained_pose.distance - successor.distance;

    double new_max_velocity =
        std::sqrt(successor.max_velocity * successor.max_velocity +
                  2. * successor.min_acceleration * ds);
    if (new_max_velocity < constrained_pose.max_velocity) {
      constrained_pose.max_velocity = new_max_velocity;

      Eigen::Vector2d linear_angular_velocity;
      linear_angular_velocity(0) =
          constrained_pose.max_velocity * (backwards ? -1. : 1.);
      linear_angular_velocity(1) = constrained_pose.max_velocity *
                                   constrained_pose.pose.curvature() *
                                   (backwards ? -1. : 1.);

      Bounds min_max_accel = model.CalculateMinMaxAcceleration(
          linear_angular_velocity, constrained_pose.pose.curvature(),
          constraints.max_voltage, high_gear);

      constrained_pose.min_acceleration =
          std::max(constrained_pose.min_acceleration,
                   backwards ? -min_max_accel.max : min_max_accel.min);

      constrained_pose.max_acceleration =
          std::min(constrained_pose.max_acceleration,
                   backwards ? -min_max_accel.min : min_max_accel.max);
    }

    successor = constrained_pose;
  }

  double t = 0.;                            // time
  double s = 0.;                            // distance
  double v = constraints.initial_velocity;  // velocity
  for (int i = 0; i < kNumSamples; i++) {
    ConstrainedPose constrained_pose = constrained_poses.at(i);
    double ds = constrained_pose.distance - s;
    double accel =
        (constrained_pose.max_velocity * constrained_pose.max_velocity -
         v * v) /
        (2. * ds);
    double dt = 0.;
    if (i > 0) {
      timed_poses_.at(i - 1).a = backwards ? -accel : accel;
      if (std::abs(accel) > 1e-6) {
        dt = (constrained_pose.max_velocity - v) / accel;
      } else if (std::abs(v) > 1e-6) {
        dt = ds / v;
      }
    }

    v = constrained_pose.max_velocity;
    s = constrained_pose.distance;
    timed_poses_.at(i) = {
        .pose = poses.at(i),
        .t = t,
        .v = backwards ? -v : v,
        .a = backwards ? -accel : accel,
    };

    t += dt;
  }
}

Trajectory::TimedPose InterpolateTime(Trajectory::TimedPose a,
                                      Trajectory::TimedPose b, double frac) {
  double new_t = a.t + ((b.t - a.t) * frac);
  double delta_t = new_t - a.t;
  if (delta_t < 0.) {
    return InterpolateTime(b, a, 1.0 - frac);
  }
  bool reversing = a.v < 0. || (::std::abs(a.v) < 1e-9 && a.a < 0.);
  double new_v = a.v + a.a * delta_t;
  double new_x_2 =
      (reversing ? -1. : 1.) * (a.v * delta_t + .5 * a.a * delta_t * delta_t);

  return {
      .pose = a.pose.Interpolate(
          b.pose, new_x_2 / (b.pose - a.pose).translational().norm()),
      .t = new_t,
      .v = new_v,
      .a = a.a,
  };
}

Trajectory::TimedPose Trajectory::SampleTime(double t) const {
  if (t <= timed_poses_.front().t) {
    return timed_poses_.front();
  } else if (t >= timed_poses_.back().t) {
    return timed_poses_.back();
  }

  for (int i = 1; i < kNumSamples; i++) {
    TimedPose sample = timed_poses_.at(i);
    if (sample.t >= t) {
      TimedPose prev_sample = timed_poses_.at(i - 1);
      if (std::abs(sample.t - prev_sample.t) < 1e-9) {
        return sample;
      } else {
        return InterpolateTime(
            prev_sample, sample,
            (t - prev_sample.t) / (sample.t - prev_sample.t));
      }
    }
  }
  return timed_poses_.at(0);
}

Trajectory::TimedPose Trajectory::Advance(double t) {
  if (progress_ == 0.) {
    current_ = timed_poses_.front();
  }
  progress_ = (t + current_.t) / timed_poses_.back().t;
  return current_ = SampleTime(current_.t + t);
}

}  // namespace control
}  // namespace muan
