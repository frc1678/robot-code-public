#ifndef MUAN_CONTROL_STATE_SPACE_CONTROLLER_H_
#define MUAN_CONTROL_STATE_SPACE_CONTROLLER_H_

#include <cstdint>
#include <limits>
#include "Eigen/Core"

namespace muan {
namespace control {

/*
 * A feedback and feedforward controller for state-space systems with
 * predefined gain matrices.
 * Example:
 *    StateSpaceController<1, 2, 1> controller{ SystemConstants::K,
 *                                              SystemConstants::Kff,
 *                                              SystemConstants::A };
 *    controller.r() = current_state;
 *    ...
 *    u = controller.Update(current_state, next_goal_state);
 */
template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
class StateSpaceController {
 public:
  // Zero-initializes all relevant matrices.
  // Using this constructor implies that the user will later manually set the
  // gain matrices to the desired values.
  StateSpaceController();

  // Initializes from a feedback gain and optional control signal constraints,
  // ignoring feedforward control.
  explicit StateSpaceController(
      const Eigen::Matrix<double, kNumInputs, kNumStates>& K,
      const Eigen::Matrix<double, kNumInputs, 1>& u_min =
          Eigen::Matrix<double, kNumInputs, 1>::Constant(
              -std::numeric_limits<double>::infinity()),
      const Eigen::Matrix<double, kNumInputs, 1>& u_max =
          Eigen::Matrix<double, kNumInputs, 1>::Constant(
              std::numeric_limits<double>::infinity()));

  // Initializes from all relevant matrices, using both feedforward and
  // feedback. A should be a discrete-time system matrix.
  StateSpaceController(const Eigen::Matrix<double, kNumInputs, kNumStates>& K,
                       const Eigen::Matrix<double, kNumInputs, kNumStates>& Kff,
                       const Eigen::Matrix<double, kNumStates, kNumStates>& A,
                       const Eigen::Matrix<double, kNumInputs, 1>& u_min =
                           Eigen::Matrix<double, kNumInputs, 1>::Constant(
                               -std::numeric_limits<double>::infinity()),
                       const Eigen::Matrix<double, kNumInputs, 1>& u_max =
                           Eigen::Matrix<double, kNumInputs, 1>::Constant(
                               std::numeric_limits<double>::infinity()));

  virtual ~StateSpaceController() = default;

  // Calculates the control signal with a possibly time-variant goal. Uses r as
  // the new goal state, calculating feedforward signal to move from the prior
  // goal to the new one with the control law:
  // u(n) = K*(r(n) - x(n)) + Kff*(r(n+1) - A*r(n))
  // This control law should be used when the goal follows a continuous,
  // dynamically-feasible motion profile.
  Eigen::Matrix<double, kNumInputs, 1> Update(
      const Eigen::Matrix<double, kNumStates, 1>& x,
      const Eigen::Matrix<double, kNumStates, 1>& r);

  // Calculates the control signal assuming a static goal with the control law:
  // u(n) = K*(r - x(n)) + Kff*(r - A*r)
  // This control law should be used when the goal does not follow a motion
  // profile but instead changes as a step function.
  Eigen::Matrix<double, kNumInputs, 1> Update(
      const Eigen::Matrix<double, kNumStates, 1>& x);

  const Eigen::Matrix<double, kNumStates, 1>& r() const;
  Eigen::Matrix<double, kNumStates, 1>& r();
  double r(uint32_t i) const;
  double& r(uint32_t i);

  const Eigen::Matrix<double, kNumInputs, 1>& u_min() const;
  Eigen::Matrix<double, kNumInputs, 1>& u_min();
  double u_min(uint32_t i) const;
  double& u_min(uint32_t i);

  const Eigen::Matrix<double, kNumInputs, 1>& u_max() const;
  Eigen::Matrix<double, kNumInputs, 1>& u_max();
  double u_max(uint32_t i) const;
  double& u_max(uint32_t i);

  const Eigen::Matrix<double, kNumInputs, kNumStates>& K() const;
  Eigen::Matrix<double, kNumInputs, kNumStates>& K();
  double K(uint32_t i, uint32_t j) const;
  double& K(uint32_t i, uint32_t j);

  const Eigen::Matrix<double, kNumInputs, kNumStates>& Kff() const;
  Eigen::Matrix<double, kNumInputs, kNumStates>& Kff();
  double Kff(uint32_t i, uint32_t j) const;
  double& Kff(uint32_t i, uint32_t j);

  const Eigen::Matrix<double, kNumStates, kNumStates>& A() const;
  Eigen::Matrix<double, kNumStates, kNumStates>& A();
  double A(uint32_t i, uint32_t j) const;
  double& A(uint32_t i, uint32_t j);

 private:
  Eigen::Matrix<double, kNumInputs, kNumStates> K_, Kff_;
  Eigen::Matrix<double, kNumStates, kNumStates> A_;
  Eigen::Matrix<double, kNumStates, 1> r_;
  Eigen::Matrix<double, kNumInputs, 1> u_min_, u_max_;
};

}  // namespace control
}  // namespace muan

#include "state_space_controller.hpp"

#endif  // MUAN_CONTROL_STATE_SPACE_CONTROLLER_H_
