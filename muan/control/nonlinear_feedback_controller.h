#ifndef MUAN_CONTROL_NONLINEAR_FEEDBACK_CONTROLLER_H_
#define MUAN_CONTROL_NONLINEAR_FEEDBACK_CONTROLLER_H_

#include <Eigen/Core>
#include "muan/control/drivetrain_model.h"
#include "muan/control/pose.h"

namespace muan {
namespace control {

class NonLinearFeedbackController {
 public:
  struct Output {
    Eigen::Vector2d velocity;      // left, right
    Eigen::Vector2d feedforwards;  // left, right
  };

  NonLinearFeedbackController(DrivetrainModel model, double beta, double zeta);
  Output Update(Eigen::Vector2d velocity, Eigen::Vector2d acceleration, Pose current, Pose error,
                bool high_gear);

 private:
  DrivetrainModel model_;
  double beta_;
  double zeta_;
};

}  // namespace control
}  // namespace muan

#endif  //  MUAN_CONTROL_NONLINEAR_FEEDBACK_CONTROLLER_H_
