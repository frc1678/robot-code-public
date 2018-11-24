#include "muan/control/nonlinear_feedback_controller.h"

namespace muan {
namespace control {

NonLinearFeedbackController::NonLinearFeedbackController(DrivetrainModel model,
                                                         double beta,
                                                         double zeta)
    : model_(model), beta_(beta), zeta_(zeta) {}

NonLinearFeedbackController::Output NonLinearFeedbackController::Update(
    Eigen::Vector2d velocity, Eigen::Vector2d acceleration, Pose current, Pose error, bool high_gear) {
  double k1 =
      2. * zeta_ *
      std::sqrt(beta_ * velocity(0) * velocity(0) + velocity(1) * velocity(1));
  double k2 = beta_;

  double err_sin = std::abs(error.heading()) < 1e-9
                       ? 1.
                       : std::sin(error.heading()) / error.heading();

  Eigen::Vector2d adjusted_velocity;

  adjusted_velocity(0) = velocity(0) * std::cos(error.heading()) +
                         k1 * ((std::cos(current.heading()) * error.Get()(0)) +
                               (std::sin(current.heading()) * error.Get()(1)));

  adjusted_velocity(1) = velocity(1) +
                         k2 * velocity(0) * err_sin *
                             (std::cos(current.heading()) * error.Get()(1) -
                              std::sin(current.heading()) * error.Get()(0)) +
                         k1 * error.heading();

  Eigen::Vector2d left_right_velocity =
      model_.InverseKinematics(adjusted_velocity);

  Eigen::Vector2d left_right_ff =
      model_.InverseDynamics(velocity, acceleration, high_gear);

  return {left_right_velocity, left_right_ff};
}

}  // namespace control
}  // namespace muan
