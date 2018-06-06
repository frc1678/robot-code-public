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

void Trajectory::SetPath(const Path &path, const State &state,
                         double velocity_final, double angular_velocity_final) {
  Reset();

  state_ = state;

  double s_min = 0.0, s_max = 1.0;
  path.Populate(s_min, s_max, &poses_[0], kNumSamples);

  states_[0] = state_;

  bool is_forward;
  // Calculate drivetrain position
  for (size_t i = 0; i < kNumSamples - 1; i++) {
    Pose delta = poses_[i + 1] - poses_[i];

    Eigen::Vector2d forward_vector;
    forward_vector << ::std::cos(poses_[i].heading()), ::std::sin(poses_[i].heading());
    double forward = delta.translational().dot(forward_vector);
    is_forward = forward > 0;

    double angular = delta.heading() * radius_;
    states_[i + 1](0) = states_[i](0) + forward - angular;
    states_[i + 1](2) = states_[i](2) + forward + angular;
  }

  if ((is_forward && state_(1) + state_(3) < -0.01) ||
      (!is_forward && state_(1) + state_(3) > 0.01)) {
    LOG(WARNING, "Conflicting path directionality: path %s and robot initial %s",
        is_forward ? "forward" : "backward",
        state_(1) + state_(3) > 0 ? "forward" : "backward");
  }
  if ((is_forward && velocity_final < -0.01) ||
      (!is_forward && velocity_final > 0.01)) {
    LOG(WARNING, "Conflicting path directionality: path %s and robot final %s",
        is_forward ? "forward" : "backward",
        velocity_final > 0 ? "forward" : "backward");
  }

  // Forward pass to calculate velocity
  for (size_t i = 0; i < kNumSamples - 1; i++) {
    const State &state_begin = states_[i];
    State &state_end = states_[i + 1];
    const State &state_next = i + 2 < kNumSamples ? states_[i + 2] : states_[i + 1];
    ConstrainAcceleration(state_begin, &state_end, state_next,
                          &segment_times_[i], false, false);
  }

  double velocity_final_left = velocity_final - angular_velocity_final * radius_;
  double velocity_final_right = velocity_final + angular_velocity_final * radius_;

  if ((states_[kNumSamples - 1](1) < velocity_final_left && is_forward) ||
      (states_[kNumSamples - 1](3) < velocity_final_right && is_forward) ||
      (states_[kNumSamples - 1](1) > velocity_final_left && !is_forward) ||
      (states_[kNumSamples - 1](3) > velocity_final_right && !is_forward)) {
    LOG(WARNING, "Unable to follow path given final speed:"
                 " final (%f, %f) and max (%f, %f)",
        velocity_final_left, velocity_final_right,
        states_[kNumSamples - 1](1), states_[kNumSamples - 1](3));
  }

  states_[kNumSamples - 1](1) = velocity_final_left;
  states_[kNumSamples - 1](3) = velocity_final_right;

  // Backwards pass to calculate velocity
  for (int i = kNumSamples - 2; i >= 0; i--) {
    State &state_begin = states_[i];
    const State &state_end = states_[i + 1];
    const State &state_before = i > 0 ? states_[i - 1] : states_[i];
    ConstrainAcceleration(state_end, &state_begin, state_before,
                          &segment_times_[i], true, true);
  }

  if (states_[0](1) > state_(1) + 0.01 ||
      states_[0](1) < state_(1) - 0.01 ||
      states_[0](3) > state_(3) + 0.01 ||
      states_[0](3) < state_(3) - 0.01) {
    LOG(WARNING, "Unable to follow path given initial speed:"
                 " starting (%f, %f) and max (%f, %f)",
        state_(1), state_(3), states_[0](1), states_[0](3));
  }

  for (size_t i = 0; i < kNumSamples - 1; i++) {
    time_remaining_ += segment_times_[i];
  }
}

void Trajectory::ConstrainOneSide(double distance, double next_distance,
                                  double input_velocity,
                                  double max_accel_voltage_same,
                                  double max_accel_voltage_other,
                                  double min_accel_voltage_same,
                                  double min_accel_voltage_other,
                                  double output_velocity_from_other_pass,
                                  double *output_velocity,
                                  bool preserve_other_pass,
                                  double opposite_accel_ratio) const {
  // Going in one direction, calculate things normally
  if ((distance >= 0 && next_distance >= 0) ||
      (distance <= 0 && next_distance <= 0)) {

    if (::std::abs(opposite_accel_ratio) > 3.0) {
      // Accel ratio is basically just to prevent both sides from ignoring
      // the other. If one side is much slower, though, it doesn't do any
      // good to limit that side further, since the time comparison will
      // fix it. If the difference is large enough, it actually slows down
      // both sides way more than necessary.
      opposite_accel_ratio = opposite_accel_ratio > 0 ? 3.0 : -3.0;
    }

    // a_opposite = k * a_same
    // a = c*B*u = c*[b d; d b]*u, assumming segment is an arc
    // u = c*invB*a = c*[b -d; -d b]*[a_same; k*a_same]
    //   = c*[b*a_same-d*k*a_same; -d*a_same+b*k*a_same]
    //   = c*[b-d*k; -d+b*k]
    // u_opposite = (b*k-d)/(b-d*k) * u_same
    double accel_voltage_ratio = (B_(1, 0) * opposite_accel_ratio - B_(1, 1)) /
                                 (B_(1, 0) - B_(1, 1) * opposite_accel_ratio);

    if (distance < 0) {
      min_accel_voltage_same =
            ::std::min(min_accel_voltage_same,
                       ::std::abs(min_accel_voltage_other / accel_voltage_ratio));

      double min_acceleration = (B_(1, 0) + B_(1, 1) * accel_voltage_ratio) *
            min_accel_voltage_same;
      if (min_acceleration > 0) {
        min_acceleration = -min_acceleration;
      }
      min_acceleration =
            ::std::min(maximum_acceleration_,
                       ::std::max(-maximum_acceleration_, min_acceleration));

      double min_output_velocity =
            StepVelocityByAcceleration(distance, input_velocity, min_acceleration);
      if (preserve_other_pass) {
        min_output_velocity = ::std::max(min_output_velocity,
                                         output_velocity_from_other_pass);
      }
      *output_velocity = min_output_velocity;
    } else {
      max_accel_voltage_same =
            ::std::min(max_accel_voltage_same,
                       ::std::abs(max_accel_voltage_other / accel_voltage_ratio));

      double max_acceleration = (B_(1, 0) + B_(1, 1) * accel_voltage_ratio) *
            max_accel_voltage_same;
      if (max_acceleration < 0) {
        max_acceleration = -max_acceleration;
      }
      max_acceleration =
            ::std::min(maximum_acceleration_,
                       ::std::max(-maximum_acceleration_, max_acceleration));

      double max_output_velocity =
            StepVelocityByAcceleration(distance, input_velocity, max_acceleration);
      if (preserve_other_pass) {
        max_output_velocity = ::std::min(max_output_velocity,
                                         output_velocity_from_other_pass);
      }
      *output_velocity = max_output_velocity;
    }
  } else {
    // On the first pass this will cause an abrupt drop in speed of one
    // side. This is fine since the second pass will fix it.
    *output_velocity = VelocityAtDirectionChange(distance, next_distance,
                                                 maximum_acceleration_);
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

  double max_accel_voltage_left;
  double max_accel_voltage_right;
  double min_accel_voltage_left;
  double min_accel_voltage_right;
  {
    double unforced_acceleration_left =
          A_(1, 1) * input_state(1) + A_(1, 3) * input_state(3);
    double unforced_acceleration_right =
          A_(1, 1) * input_state(3) + A_(1, 3) * input_state(1);

    // Bu = -Ax
    // u = invB*(-unforced) = [b -d; -d b]/(b^2-b^2)*(-unforced)
    double holding_left = (unforced_acceleration_right * B_(1, 1) -
                           unforced_acceleration_left * B_(1, 0)) * invB_scale_;
    double holding_right = (unforced_acceleration_left * B_(1, 1) -
                            unforced_acceleration_right * B_(1, 0)) * invB_scale_;
    if (reverse_pass) {
      holding_left = -holding_left;
      holding_right = -holding_right;
    }

    max_accel_voltage_left = maximum_voltage_ - holding_left;
    max_accel_voltage_right = maximum_voltage_ - holding_right;
    min_accel_voltage_left = -maximum_voltage_ - holding_left;
    min_accel_voltage_right = -maximum_voltage_ - holding_right;
  }

  // Assume segment is an arc, so aL/aR = xL/xR
  ConstrainOneSide(distance_left, distance_next_left, input_state(1),
                   max_accel_voltage_left, max_accel_voltage_right,
                   min_accel_voltage_left, min_accel_voltage_right,
                   (*output_state)(1), &(*output_state)(1), preserve_other_pass,
                   distance_right / distance_left);
  ConstrainOneSide(distance_right, distance_next_right, input_state(3),
                   max_accel_voltage_right, max_accel_voltage_left,
                   min_accel_voltage_right, min_accel_voltage_left,
                   (*output_state)(3), &(*output_state)(3), preserve_other_pass,
                   distance_left / distance_right);

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
  time_remaining_ = ::std::max(time_remaining_ - 0.005, 0.0);
  size_t index = last_index_;

  while (index < kNumSamples - 1 && time_since_index_ > segment_times_[index]) {
    time_since_index_ -= segment_times_[index];
    index++;
  }

  last_index_ = index;

  // When complete maintain speed until replaced by new path
  double left_acceleration = 0;
  double right_acceleration = 0;
  if (index < kNumSamples - 1) {
    left_acceleration = (states_[index + 1](1) - states_[index](1)) / segment_times_[index];
    right_acceleration = (states_[index + 1](3) - states_[index](3)) / segment_times_[index];
  }

  // Interpolate between the states using kinematics
  double left_distance = states_[index](0) + states_[index](1) * time_since_index_ +
                           0.5 * left_acceleration * time_since_index_ * time_since_index_;
  double left_velocity = states_[index](1) + left_acceleration * time_since_index_;

  double right_distance = states_[index](2) + states_[index](3) * time_since_index_ +
                            0.5 * right_acceleration * time_since_index_ * time_since_index_;
  double right_velocity = states_[index](3) + right_acceleration * time_since_index_;

  state_ = (Eigen::Matrix<double, 4, 1>() << left_distance, left_velocity,
          right_distance, right_velocity).finished();

  double distance_in = 0.5 *
          (left_distance + right_distance - states_[index](0) - states_[index](2));
  double current_angle = 0.5 * (right_distance - left_distance) / radius_;
  double current_x = poses_[index].Get()(0) +
          0.5 * distance_in * (::std::cos(current_angle) + ::std::cos(poses_[index].Get()(2)));
  double current_y = poses_[index].Get()(1) +
          0.5 * distance_in * (::std::sin(current_angle) + ::std::sin(poses_[index].Get()(2)));

  Pose pose((Position() << current_x, current_y).finished(), current_angle);

  bool profile_complete = index == kNumSamples - 1;
  double distance_remaining = 0.5 * (states_[kNumSamples - 1](0) - left_distance +
                                     states_[kNumSamples - 1](2) - right_distance);
  return Sample{pose, state_, distance_remaining, time_remaining_, profile_complete};
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
  time_remaining_ = 0.0;
}

}  // namespace paths
}  // namespace control_loops
}  // namespace frc971
