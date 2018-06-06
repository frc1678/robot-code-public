#ifndef FRC971_CONTROL_LOOPS_PATHS_TRAJECTORY_H_
#define FRC971_CONTROL_LOOPS_PATHS_TRAJECTORY_H_

#include "Eigen/Core"
#include "third_party/frc971/control_loops/paths/path.h"

namespace frc971 {

namespace control_loops {

namespace paths {

constexpr size_t kNumSamples = 1001;

double StepVelocityByAcceleration(double distance, double initial_velocity, double acceleration);

double VelocityAtDirectionChange(double distance1, double distance2, double acceleration);

// State is (left pos, left vel, right pos, right vel)
using State = Eigen::Matrix<double, 4, 1>;

class Trajectory {
 public:
  struct Sample {
    Pose pose;
    State drivetrain_state;
    double distance_remaining;
    double time_remaining;
    bool profile_complete;
  };

  // Pass in a nullptr for no path
  void SetPath(const Path &path, const State &state,
               double veloicty_final, double angular_velocity_final);

  // Update at 200hz, returning the new sample
  Sample Update();

  void MoveCurrentState(const State &current);

  inline void set_maximum_acceleration(double maximum_acceleration) {
    maximum_acceleration_ = maximum_acceleration;
  }

  inline void set_maximum_voltage(double maximum_voltage) {
    maximum_voltage_ = maximum_voltage;
  }

  inline void set_system(const Eigen::Matrix<double, 4, 4> &A_c,
                         const Eigen::Matrix<double, 4, 2> &B_c,
                         double radius) {
    A_ = A_c;
    B_ = B_c;
    invB_scale_ = 1 / (B_(1, 0) * B_(1, 0) - B_(1, 1) * B_(1, 1));
    radius_ = radius;
  }

  void Reset();

  ::std::array<Pose, kNumSamples> poses_;
  ::std::array<State, kNumSamples> states_;
  ::std::array<double, kNumSamples - 1> segment_times_;

 private:
  void ConstrainAcceleration(const State &state_begin, State *state_end, const State &ref_state,
                             double *segment_time, bool reverse_pass, bool preserve_other_pass) const;

  void ConstrainOneSide(double distance, double next_distance,
                        double velocity_initial,
                        double max_accel_voltage_same,
                        double max_accel_voltage_other,
                        double min_accel_voltage_same,
                        double min_accel_voltage_other,
                        double velocity_final_from_other_pass,
                        double *velocity_final,
                        bool preserve_other_pass,
                        double opposite_accel_ratio) const;

  double maximum_acceleration_;
  double maximum_voltage_;

  Eigen::Matrix<double, 4, 4> A_;
  Eigen::Matrix<double, 4, 2> B_;
  double invB_scale_;
  double radius_;

  // The index of the last _segment_ that was used. This segment might be used
  // again.
  size_t last_index_ = 0;
  // The time already passed _on the current segment only_.
  double time_since_index_ = 0.0;

  double time_remaining_ = 0.0;

  State state_;
};

}  // namespace paths
}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_PATHS_TRAJECTORY_H_
