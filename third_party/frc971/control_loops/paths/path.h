#ifndef FRC971_CONTROL_LOOPS_PATHS_PATH_H_
#define FRC971_CONTROL_LOOPS_PATHS_PATH_H_

#include "Eigen/Core"

namespace frc971 {

namespace control_loops {

namespace paths {

using Position = Eigen::Vector2d;

class Pose {
 public:
  Pose() = default;
  Pose(Position position, double theta);
  Pose(Eigen::Vector3d values);

  Pose operator+(const Pose &other) const;
  Pose operator-(const Pose &other) const;

  inline Eigen::Vector3d Get() const { return values_; }
  inline Eigen::Vector2d translational() const {
    return values_.block<2, 1>(0, 0);
  }
  inline double heading() const { return values_(2); }

  Pose TranslateBy(const Position &delta) const;
  Pose RotateBy(double theta) const;

  // Compose this pose with another. Treat the new pose as an offset, using this
  // pose as the origin (theta=x axis)
  Pose Compose(const Pose &other) const;

 private:
  Eigen::Vector3d values_;
};

class Path {
 public:
  // Populate `pose_arr` with values (if it isn't nullptr). `pose_arr` should
  // have `num_samples` entries.
  virtual void Populate(double s_min, double s_max, Pose *pose_arr,
                        size_t arr_len) const = 0;
};

class HermitePath : public Path {
 public:
  HermitePath(Pose initial, Pose final, double initial_velocity,
              double final_velocity, bool backwards);
  HermitePath(Position initial_position, Eigen::Vector2d initial_tangent,
              Position final_position, Eigen::Vector2d final_tangent,
              double initial_velocity, double final_velocity, bool backwards);

  virtual void Populate(double s_min, double s_max, Pose *pose_arr,
                        size_t arr_len) const override;

 private:
  // (1, s, s^2, s^3, s^4, s^5) -> (x, y, x', y')
  Eigen::Matrix<double, 4, 6> coefficients_;

  // Cached, because it is lost in the first-derivative at s=0
  double initial_heading_;

  bool backwards_;
};

}  // namespace paths
}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_PATHS_PATH_H_
