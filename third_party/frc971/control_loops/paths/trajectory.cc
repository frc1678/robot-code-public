#include "third_party/frc971/control_loops/paths/trajectory.h"
#include <cmath>
#include "third_party/aos/common/die.h"
#include "muan/logging/logger.h"

namespace frc971 {

namespace control_loops {

namespace paths {

double StepVelocityByAcceleration(double distance, double initial_velocity, double acceleration) {
  if (distance == 0) {
    return initial_velocity;
  }
  if ((distance > 0 && initial_velocity <= 0 && acceleration <= 0) ||
      (distance < 0 && initial_velocity >= 0 && acceleration >= 0)) {
    // There is no solution that takes positive time
    return NAN;
  }
  // vf^2 = v0^2 + 2*a*d
  double velocity = ::std::sqrt(initial_velocity * initial_velocity + 2 * acceleration * distance);
  // Take the direct path, handling change in direction is done elsewhere
  if (distance > 0) {
    return velocity;
  } else {
    return -velocity;
  }
}

double VelocityAtDirectionChange(double distance1, double distance2, double acceleration) {
  // The control points are approximately evenly spread in time across
  // a sufficiently small interval
  // x(i) = a*i^2 + b*i, x(1) = distance1, x(2) = distance1 + distance2
  double a = -0.5 * distance1 + 0.5 * distance2;
  // x'(t) = x(2*t/total_t)
  // acceleration = 8*a/time^2
  double time = ::std::sqrt(8 * ::std::abs(a / acceleration));
  // velocity of center is mean velocity
  return (distance1 + distance2) / time;
}

void Trajectory::SetPath(const Path &path, const State &state) {
  Reset();

  state_ = state;

  double s_min = 0.0, s_max = 1.0;
  path.Populate(s_min, s_max, &poses_[0], kNumSamples);

  states_[0] = state_;

  // Calculate drivetrain position
  for (size_t i = 0; i < kNumSamples - 1; i++) {
    Pose delta = poses_[i + 1] - poses_[i];

    Eigen::Vector2d forward_vector;
    forward_vector << ::std::cos(poses_[i].heading()), ::std::sin(poses_[i].heading());
    double forward = delta.translational().dot(forward_vector);

    if ((forward > 0 && state_(1) + state_(3) < -0.01) ||
        (forward < 0 && state_(1) + state_(3) > 0.01)) {
      ::aos::Die("Conflicting path directionality: path %s and robot %s",
                 forward > 0 ? "forward" : "backward",
                 state_(1) + state_(3) > 0 ? "forward" : "backward");
    }

    double angular = delta.heading() * radius_;
    states_[i + 1](0) = states_[i](0) + forward - angular;
    states_[i + 1](2) = states_[i](2) + forward + angular;
  }

  // Forward pass to calculate velocity
  for (size_t i = 0; i < kNumSamples - 1; i++) {
    const State &state_begin = states_[i];
    State &state_end = states_[i + 1];
    const State &state_next = i + 2 < kNumSamples ? states_[i + 2] : states_[i + 1];
    ConstrainAcceleration(state_begin, &state_end, state_next, &segment_times_[i], false, false);
  }

  states_[kNumSamples - 1](1) = states_[kNumSamples - 1](3) = 0.0;

  // Backwards pass to calculate velocity
  for (int i = kNumSamples - 2; i >= 0; i--) {
    State &state_begin = states_[i];
    const State &state_end = states_[i + 1];
    const State &state_before = i > 0 ? states_[i - 1] : states_[i];
    ConstrainAcceleration(state_end, &state_begin, state_before, &segment_times_[i], true, true);
  }

  if (states_[0](1) > state_(1) + 0.01 ||
      states_[0](1) < state_(1) - 0.01 ||
      states_[0](3) > state_(3) + 0.01 ||
      states_[0](3) < state_(3) - 0.01) {
    LOG(WARNING, "WARNING: Unable to follow path given initial speed: starting (%f, %f) and max (%f, %f)",
          state_(1), state_(3), states_[0](1), states_[0](3));
  }
}

void Trajectory::ConstrainOneSide(double distance, double next_distance,
                                  double input_velocity, double other_input_velocity,
                                  double output_velocity_from_other_pass, double *output_velocity,
                                  bool reverse_pass, bool preserve_other_pass) const {
  constexpr double kMaxVoltage = 9.0;

  double unforced_acceleration = A_(1, 1) * input_velocity + A_(1, 3) * other_input_velocity;
  if (reverse_pass) {
    unforced_acceleration *= -1;
  }
  double input_acceleration = (B_(1, 0) + B_(1, 1)) * kMaxVoltage;

  double min_acceleration = ::std::max(-maximum_acceleration_, unforced_acceleration - input_acceleration);
  double max_acceleration = ::std::min(maximum_acceleration_, unforced_acceleration + input_acceleration);

  // Going in one direction, calculate things normally
  if ((distance >= 0 && next_distance >= 0) ||
      (distance <= 0 && next_distance <= 0)) {
    double min_output_velocity = StepVelocityByAcceleration(distance, input_velocity, min_acceleration);

    double max_output_velocity = StepVelocityByAcceleration(distance, input_velocity, max_acceleration);

    if (preserve_other_pass) {
      min_output_velocity = ::std::max(min_output_velocity, output_velocity_from_other_pass);
      max_output_velocity = ::std::min(max_output_velocity, output_velocity_from_other_pass);
    }

    if (distance < 0) {
      *output_velocity = min_output_velocity;
    } else {
      *output_velocity = max_output_velocity;
    }
  } else {
    // On the first pass this will cause an abrupt drop in speed of one
    // side. This is fine since the second pass will fix it.
    *output_velocity = VelocityAtDirectionChange(distance, next_distance, maximum_acceleration_);
  }
}

void Trajectory::ConstrainAcceleration(const State &input_state, State *output_state,
                                       const State &ref_state, double *segment_time,
                                       bool reverse_pass, bool preserve_other_pass) const {
  double distance_left = (*output_state)(0) - input_state(0);
  double distance_next_left = ref_state(0) - (*output_state)(0);
  double distance_right = (*output_state)(2) - input_state(2);
  double distance_next_right = ref_state(2) - (*output_state)(2);
  // On reverse pass, output_state occurs sooner than input_state, so the
  // sign on the delta needs to be fixed.
  if (reverse_pass) {
    distance_left *= -1;
    distance_next_left *= -1;
    distance_right *= -1;
    distance_next_right *= -1;
  }
  ConstrainOneSide(distance_left, distance_next_left, input_state(1), input_state(3),
                   (*output_state)(1), &(*output_state)(1), reverse_pass, preserve_other_pass);
  ConstrainOneSide(distance_right, distance_next_right, input_state(3), input_state(1),
                   (*output_state)(3), &(*output_state)(3), reverse_pass, preserve_other_pass);

  double time_left = distance_left * 2 / (input_state(1) + (*output_state)(1));
  double time_right = distance_right * 2 / (input_state(3) + (*output_state)(3));

  // On the pose the drivetrain crosses the v=0 mark, average velocity
  // could be very imprecise, so ignore it
  if ((input_state(1) > 0) != ((*output_state)(1) > 0)) {
    *segment_time = time_right;
  } else if ((input_state(3) > 0) != ((*output_state)(3) > 0)) {
    *segment_time = time_left;
  } else {
    *segment_time = ::std::max(time_left, time_right);
  }

  // Slow down the side that takes less time, so both sides take the same
  // amount of time. Balance the output velocity instead of the average
  // velocity to prevent errors from percolating. This doesn't cause
  // significant issues because the poses are close enough that the path
  // between them is essentially an arc.
  if (time_left < *segment_time) {
    (*output_state)(1) *= time_left / *segment_time;
  }
  if (time_right < *segment_time) {
    (*output_state)(3) *= time_right / *segment_time;
  }
}

Trajectory::Sample Trajectory::Update() {
  time_since_index_ += 0.005;
  size_t index = last_index_;

  while (index < kNumSamples - 1 && time_since_index_ > segment_times_[index]) {
    time_since_index_ -= segment_times_[index];
    index++;
  }

  last_index_ = index;

  if (index < kNumSamples - 1) {
    // Interpolate between the states using kinematics
    double left_acceleration = (states_[index + 1](1) - states_[index](1)) / segment_times_[index];
    double right_acceleration = (states_[index + 1](3) - states_[index](3)) / segment_times_[index];

    double left_distance = states_[index](0) + states_[index](1) * time_since_index_ +
                             0.5 * left_acceleration * time_since_index_ * time_since_index_;
    double left_velocity = states_[index](1) + left_acceleration * time_since_index_;

    double right_distance = states_[index](2) + states_[index](3) * time_since_index_ +
                              0.5 * right_acceleration * time_since_index_ * time_since_index_;
    double right_velocity = states_[index](3) + right_acceleration * time_since_index_;

    state_ = (Eigen::Matrix<double, 4, 1>() << left_distance, left_velocity,
            right_distance, right_velocity).finished();
    return Sample{Pose{}, state_};
  } else {
    // Already at end of trajectory
    return Sample{Pose{}, states_[kNumSamples - 1]};
  }
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
  time_since_index_ = 0.0;
}

}  // namespace paths
}  // namespace control_loops
}  // namespace frc971
