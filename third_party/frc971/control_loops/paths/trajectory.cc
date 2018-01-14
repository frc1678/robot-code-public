#include "third_party/frc971/control_loops/paths/trajectory.h"
#include <iostream>

namespace frc971 {

namespace control_loops {

namespace paths {

void Trajectory::SetPath(const Path &path, const State &state) {
  Reset();

  state_ = state;

  double s_min = 0.0, s_max = 1.0;
  path.Populate(s_min, s_max, &poses_[0], kNumSamples);

  for (size_t i = 0; i < kNumSamples - 1; i++) {
    // Calculate maximum velocity given curvature
    Pose delta = poses_[i + 1] - poses_[i];
    double curvature = ::std::abs(delta.heading() / delta.translational().norm());
    states_[i](1) = 1.0 / (1.0 / maximum_velocity_ + curvature / maximum_angular_velocity_);
    states_[i](3) = curvature * states_[i](1);
  }

  // Backwards pass
  states_[kNumSamples - 1](1) = states_[kNumSamples - 1](3) = 0.0;
  for (int i = kNumSamples - 2; i >= 0; i--) {
    const Pose &segment_begin = poses_[i + 1];
    const Pose &segment_end = poses_[i];
    const State &state_begin = states_[i + 1];
    State &state_end = states_[i];
    ConstrainAcceleration(segment_begin, segment_end, state_begin, &state_end, &segment_times_[i], false);
  }

  // Do the forwards pass last so that the state heading calculations don't stay
  // messed up (reversed in backwards pass).
  states_[0] = state_;
  states_[0](2) = poses_[0].heading();
  for (size_t i = 0; i < kNumSamples - 1; i++) {
    const Pose &segment_begin = poses_[i];
    const Pose &segment_end = poses_[i + 1];
    const State &state_begin = states_[i];
    State &state_end = states_[i + 1];
    ConstrainAcceleration(segment_begin, segment_end, state_begin, &state_end, &segment_times_[i], true);
  }
}

void Trajectory::ConstrainAcceleration(const Pose &segment_begin, const Pose &segment_end,
                                       const State &state_begin, State *state_end,
                                       double *segment_time, bool reversed) const {
  // The reparametrization step
  double initial_forward_velocity = state_begin(1);
  double initial_angular_velocity = state_begin(3);
  double distance = (segment_end - segment_begin).translational().norm();
  double delta_heading = (segment_end - segment_begin).heading();

  double curvature = ::std::abs(delta_heading / distance);

  double acceleration = maximum_acceleration_;
  double angular_acceleration = maximum_angular_acceleration_;

  if (curvature * acceleration < angular_acceleration) {
    angular_acceleration = curvature * acceleration;
  } else {
    acceleration = angular_acceleration / curvature;
  }

  double final_forward_velocity =
      ::std::sqrt(initial_forward_velocity * initial_forward_velocity + 2 * acceleration * distance);

  final_forward_velocity = ::std::min(final_forward_velocity, (*state_end)(1));

  double final_angular_velocity =
      ::std::sqrt(initial_angular_velocity * initial_angular_velocity + 2 * angular_acceleration * distance);

  final_angular_velocity = ::std::min(final_angular_velocity, ::std::abs((*state_end)(3)));

  /* if (delta_heading / (final_angular_velocity + initial_angular_velocity) < 0) { */
  /*   final_angular_velocity = -final_angular_velocity; */
  /* } */
  
  /*
  if (reversed) {
    final_angular_velocity = -final_angular_velocity;
  }
  */

  *segment_time = ::std::abs(2 * distance / (initial_forward_velocity + final_forward_velocity));
  (*state_end)(0) = state_begin(0) + distance;
  (*state_end)(1) = final_forward_velocity;
  (*state_end)(2) = segment_end.heading();
  (*state_end)(3) = final_angular_velocity;
}

Trajectory::Sample Trajectory::Update() {
  time_to_go_ += 0.005;
  size_t index = last_index_;

  while (index < kNumSamples - 2 && time_to_go_ > segment_times_[index]) {
    time_to_go_ -= segment_times_[index];
    index++;
  }

  if (time_to_go_ > segment_times_[index]) {
    time_to_go_ = segment_times_[index];
  }

  // Interpolate between the states using kinematics
  double acceleration = (states_[index + 1](1) - states_[index](1)) / segment_times_[index];
  double distance =
      states_[index](0) + states_[index](1) * time_to_go_ + 0.5 * acceleration * time_to_go_ * time_to_go_;
  double velocity = states_[index](1) + acceleration * time_to_go_;

  double angular_acceleration = (states_[index + 1](3) - states_[index](3)) / segment_times_[index];
  double heading = states_[index](2) + states_[index](3) * time_to_go_ +
                   0.5 * angular_acceleration * time_to_go_ * time_to_go_;
  double angular_velocity = states_[index](3) + angular_acceleration * time_to_go_; // (states_[index + 1](2) - states_[index](2)) / segment_times_[index];

  if (states_[index + 1](2) < states_[index](2)) {
    angular_velocity = -angular_velocity;
  }

  last_index_ = index;

  state_ = (Eigen::Matrix<double, 4, 1>() << distance, velocity, heading, angular_velocity).finished();
  return Sample{Pose{}, state_};
}

void Trajectory::Reset() {
  state_ = {};

  for (size_t i = 0; i < kNumSamples - 1; i++) {
    poses_[i] = {};
    states_[i] = State::Zero();
    segment_times_[i] = 0.0;
  }

  poses_[kNumSamples - 1] = {};
  states_[kNumSamples - 1] = State::Zero();

  last_index_ = 0;
  time_to_go_ = 0.0;
}

}  // namespace paths
}  // namespace control_loops
}  // namespace frc971
