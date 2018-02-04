#ifndef FRC971_CONTROL_LOOPS_PATHS_TRAJECTORY_H_
#define FRC971_CONTROL_LOOPS_PATHS_TRAJECTORY_H_

#include "Eigen/Core"
#include "third_party/frc971/control_loops/paths/path.h"

namespace frc971 {

namespace control_loops {

namespace paths {

constexpr size_t kNumSamples = 1001;

double StepVelocityByAcceleration(double distance, double initial_velocity, double acceleration);

// State is (left pos, left vel, right pos, right vel)
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
  inline void set_system(const Eigen::Matrix<double, 4, 4> &A_c, const Eigen::Matrix<double, 4, 2> B_c,
                         double radius) {
    A_ = A_c;
    B_ = B_c;
    radius_ = radius;
  }

  void Reset();

  ::std::array<Pose, kNumSamples> poses_;
  ::std::array<State, kNumSamples> states_;
  ::std::array<double, kNumSamples - 1> segment_times_;

 private:
  void ConstrainAcceleration(const State &state_begin, State *state_end, double *segment_time,
                             bool reverse_pass, bool preserve_other_pass) const;

  void ConstrainOneSide(double distance, double velocity_initial, double velocity_other_initial,
                        double velocity_final_from_other_pass, double *velocity_final,
                        bool reverse_pass, bool preserve_other_pass) const;

  double maximum_acceleration_;

  Eigen::Matrix<double, 4, 4> A_;
  Eigen::Matrix<double, 4, 2> B_;
  double radius_;

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
