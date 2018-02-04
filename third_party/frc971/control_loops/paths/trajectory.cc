#include "third_party/frc971/control_loops/paths/trajectory.h"
#include "third_party/aos/common/die.h"
#include "muan/logging/logger.h"

namespace frc971 {

namespace control_loops {

namespace paths {

double StepVelocityByAcceleration(double distance, double initial_velocity, double acceleration) {
  double velocity = ::std::sqrt(initial_velocity * initial_velocity + 2 * acceleration * distance);
  if (distance / (initial_velocity + velocity) < 0) {
    velocity = -velocity;
  }
  return velocity;
}

void Trajectory::SetPath(const Path &path, const State &state) {
  Reset();

  state_ = state;

  double s_min = 0.0, s_max = 1.0;
  path.Populate(s_min, s_max, &poses_[0], kNumSamples);

  states_[0] = state_;
  for (size_t i = 0; i < kNumSamples - 1; i++) {
    // Calculate maximum velocity given curvature
    Pose delta = poses_[i + 1] - poses_[i];

    Eigen::Vector2d forward_vector;
    forward_vector << ::std::cos(poses_[i].heading()), ::std::sin(poses_[i].heading());
    double forward = delta.translational().dot(forward_vector);

    if (forward > 0 && state_(1) + state_(3) < -0.01 ||
        forward < 0 && state_(1) + state_(3) > 0.01) {
      ::aos::Die("Conflicting path directionality: path %s and robot %s",
                 forward > 0 ? "forward" : "backward",
                 state_(1) + state_(3) > 0 ? "forward" : "backward");
    }

    double angular = delta.heading() * radius_;
    states_[i + 1](0) = states_[i](0) + forward - angular;
    states_[i + 1](2) = states_[i](2) + forward + angular;
  }

  for (size_t i = 0; i < kNumSamples - 1; i++) {
    const State &state_begin = states_[i];
    State &state_end = states_[i + 1];
    ConstrainAcceleration(state_begin, &state_end, &segment_times_[i], false, false);
  }

  // Backwards pass
  states_[kNumSamples - 1](1) = states_[kNumSamples - 1](3) = 0.0;
  for (int i = kNumSamples - 2; i >= 0; i--) {
    State &state_begin = states_[i];
    const State &state_end = states_[i + 1];
    ConstrainAcceleration(state_end, &state_begin, &segment_times_[i], true, true);
  }

  if (states_[0](1) > state_(1) + 0.01 ||
      states_[0](1) < state_(1) - 0.01 ||
      states_[0](3) > state_(3) + 0.01 ||
      states_[0](3) < state_(3) - 0.01) {
    LOG_P("WARNING: Unable to follow path given initial speed: starting (%f, %f) and max (%f, %f)",
          state_(1), state_(3), states_[0](1), states_[0](3));
  }
}

void Trajectory::ConstrainOneSide(double distance, double velocity_initial, double velocity_other_initial,
                                  double velocity_final_from_other_pass, double *velocity_final,
                                  bool reverse_pass, bool preserve_other_pass) const {
  constexpr double kMaxVoltage = 10.0;

  double unforced_acceleration = A_(1, 1) * velocity_initial + A_(1, 3) * velocity_other_initial;
  double input_acceleration = (B_(1, 0) + B_(1, 1)) * kMaxVoltage;

  double min_acceleration = -maximum_acceleration_;
  double max_acceleration = maximum_acceleration_;

  if (!reverse_pass) {
    min_acceleration = ::std::max(min_acceleration, unforced_acceleration - input_acceleration);
    max_acceleration = ::std::min(max_acceleration, unforced_acceleration + input_acceleration);
  }

  double min_velocity_final = StepVelocityByAcceleration(distance, velocity_initial, min_acceleration);

  double max_velocity_final = StepVelocityByAcceleration(distance, velocity_initial, max_acceleration);

  if (preserve_other_pass) {
    min_velocity_final = ::std::max(min_velocity_final, velocity_final_from_other_pass);
    max_velocity_final = ::std::min(max_velocity_final, velocity_final_from_other_pass);
  }

  if (distance < 0) {
    *velocity_final = min_velocity_final;
  } else {
    *velocity_final = max_velocity_final;
  }
}

void Trajectory::ConstrainAcceleration(const State &state_begin, State *state_end, double *segment_time,
                                       bool reverse_pass, bool preserve_other_pass) const {
  double distance_left = (*state_end)(0) - state_begin(0);
  double distance_right = (*state_end)(2) - state_begin(2);
  if (reverse_pass) {
    distance_left *= -1;
    distance_right *= -1;
  }
  ConstrainOneSide(distance_left, state_begin(1), state_begin(3),
                   (*state_end)(1), &(*state_end)(1), reverse_pass, preserve_other_pass);
  ConstrainOneSide(distance_right, state_begin(3), state_begin(1),
                   (*state_end)(3), &(*state_end)(3), reverse_pass, preserve_other_pass);

  double average_left_velocity = 0.5 * (state_begin(1) + (*state_end)(1));
  double average_right_velocity = 0.5 * (state_begin(3) + (*state_end)(3));
  if (::std::abs(distance_left * average_right_velocity) <
      ::std::abs(distance_right * average_left_velocity)) {
    average_left_velocity = distance_left * average_right_velocity / distance_right;
    (*state_end)(1) = 2 * average_left_velocity - state_begin(1);
  } else if (::std::abs(distance_left * average_right_velocity) >
             ::std::abs(distance_right * average_left_velocity)) {
    average_right_velocity = distance_right * average_left_velocity / distance_left;
    (*state_end)(3) = 2 * average_right_velocity - state_begin(3);
  }

  *segment_time = (distance_left + distance_right) /
                  (average_left_velocity + average_right_velocity);
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
  double left_acceleration = (states_[index + 1](1) - states_[index](1)) / segment_times_[index];
  double left_distance = states_[index](0) + states_[index](1) * time_to_go_ +
                         0.5 * left_acceleration * time_to_go_ * time_to_go_;
  double left_velocity = states_[index](1) + left_acceleration * time_to_go_;

  double right_acceleration = (states_[index + 1](3) - states_[index](3)) / segment_times_[index];
  double right_distance = states_[index](2) + states_[index](3) * time_to_go_ +
                          0.5 * right_acceleration * time_to_go_ * time_to_go_;
  double right_velocity = states_[index](3) + right_acceleration * time_to_go_;

  last_index_ = index;

  state_ = (Eigen::Matrix<double, 4, 1>() << left_distance, left_velocity, right_distance, right_velocity)
               .finished();
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
