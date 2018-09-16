#ifndef MUAN_CONTROL_TRAJECTORY_H_
#define MUAN_CONTROL_TRAJECTORY_H_

#include <algorithm>
#include <vector>
#include "muan/control/drivetrain_model.h"
#include "muan/control/spline.h"

namespace muan {
namespace control {

class Trajectory {
 public:
  struct Constraints {
    double max_velocity;
    double max_voltage;
    double max_acceleration;
    double max_centripetal_acceleration;

    double initial_velocity;
    double final_velocity;
  };

  struct ConstrainedPose {
    PoseWithCurvature pose;
    double distance;
    double max_velocity;
    double min_acceleration;
    double max_acceleration;
  };

  struct TimedPose {
    PoseWithCurvature pose;
    double t;
    double v;
    double a;
  };

  Trajectory() = default;
  Trajectory(const HermiteSpline& spline, Constraints constraints,
             bool high_gear, const DrivetrainModel& model);

  TimedPose SampleTime(double t) const;

  TimedPose Advance(double t);
  inline bool done() const { return 1 - progress_ < 1e-6; }

  void TimeReparametrize(const HermiteSpline& spline,
                         const DrivetrainModel& model, bool high_gear,
                         Constraints constraints);

 private:
  std::array<TimedPose, kNumSamples> timed_poses_;

  double progress_ = 0.;
  TimedPose current_;
};

}  // namespace control
}  // namespace muan

#endif  // MUAN_CONTROL_TRAJECTORY_H_
