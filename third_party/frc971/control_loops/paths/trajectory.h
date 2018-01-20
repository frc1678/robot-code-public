#ifndef FRC971_CONTROL_LOOPS_PATHS_TRAJECTORY_H_
#define FRC971_CONTROL_LOOPS_PATHS_TRAJECTORY_H_

#include "Eigen/Core"
#include "third_party/frc971/control_loops/paths/path.h"

namespace frc971 {

namespace control_loops {

namespace paths {

constexpr size_t kNumSamples = 1001;

// State in (arc length, forward velocity, heading, angular velocity)
using State = Eigen::Matrix<double, 4, 1>;

class Trajectory {
 public:
  struct Sample {
    Pose pose;

    State drivetrain_state;
  };

  // Pass in a nullptr for no path
  void SetPath(const Path &path, const State &state);

  // Update at 200hz, returning the new sample
  Sample Update();

  void MoveCurrentState(const State &current);

  inline void set_maximum_acceleration(double maximum_acceleration) {
    maximum_acceleration_ = maximum_acceleration;
  }
  inline void set_maximum_angular_acceleration(double maximum_angular_acceleration) {
    maximum_angular_acceleration_ = maximum_angular_acceleration;
  }
  inline void set_maximum_velocity(double maximum_velocity) {
    maximum_velocity_ = maximum_velocity;
  }
  inline void set_maximum_angular_velocity(double maximum_angular_velocity) {
    maximum_angular_velocity_ = maximum_angular_velocity;
  }

  void Reset();

  ::std::array<Pose, kNumSamples> poses_;
  ::std::array<State, kNumSamples> states_;
  ::std::array<double, kNumSamples - 1> segment_times_;

 private:
  void ConstrainAcceleration(const Pose &segment_begin, const Pose &segment_end,
                             const State &state_begin, State *state_end,
                             double *segment_time, bool reversed) const;

  double maximum_acceleration_;
  double maximum_angular_acceleration_;
  double maximum_velocity_;
  double maximum_angular_velocity_;

  // The index of the last _segment_ that was used. This segment might be used
  // again.
  size_t last_index_ = 0;
  // The time already passed _on the current segment only_.
  double time_to_go_ = 0.0;

  State state_;
};

}  // namespace paths
}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_PATHS_TRAJECTORY_H_
