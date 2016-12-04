#ifndef MUAN_CONTROL_STATE_SPACE_OBSERVER_H_
#define MUAN_CONTROL_STATE_SPACE_OBSERVER_H_

#include "Eigen/Core"
#include "state_space_plant.h"
#include <cstdint>

namespace muan {

namespace control {

/*
 * A linear estimator for state-space systems using predefined gain matrices.
 * Example:
 *    StateSpacePlant<1, 2, 1> plant{ SystemConstants::A,
 *                                    SystemConstants::B,
 *                                    SystemConstants::C,
 *                                    SystemConstants::D,
 *                                    initial_state };
 *    StateSpaceObserver<1, 2, 1> observer{ plant, SystemConstants::L };
 *    ...
 *    observer.Update(control_signal, measurement);
 */
template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
class StateSpaceObserver {
 public:
  // Initializes the system to sane defaults and the gains matrix to zero. Using
  // this constructor implies that the system and gains matrices will be set at
  // a later time.
  StateSpaceObserver();

  // Initializes the observer from a plant and discrete-time observer gains.
  explicit StateSpaceObserver(
      const muan::control::StateSpacePlant<kNumInputs, kNumStates, kNumOutputs>& plant,
      Eigen::Matrix<double, kNumStates, kNumOutputs> L =
          Eigen::Matrix<double, kNumStates, kNumOutputs>::Zero());
  virtual ~StateSpaceObserver() = default;

  // Update the observer's estimate with new control signal data and measurement
  // data with a difference equation as follows:
  // xhat_post(n) = xhat(n) + L * (y(n) - C*xhat(n))
  // xhat(n+1) = A*xhat_post(n) + B*u(n)
  void Update(const Eigen::Matrix<double, kNumInputs, 1>& u, const Eigen::Matrix<double, kNumOutputs, 1>& y);

  const Eigen::Matrix<double, kNumStates, 1>& x() const;
  Eigen::Matrix<double, kNumStates, 1>& x();
  double x(uint32_t i) const;
  double& x(uint32_t i);

  const muan::control::StateSpacePlant<kNumInputs, kNumStates, kNumOutputs>& plant() const;
  muan::control::StateSpacePlant<kNumInputs, kNumStates, kNumOutputs>& plant();

  const Eigen::Matrix<double, kNumStates, kNumOutputs>& L() const;
  Eigen::Matrix<double, kNumStates, kNumOutputs>& L();
  double L(uint32_t i, uint32_t j) const;
  double& L(uint32_t i, uint32_t j);

 private:
  muan::control::StateSpacePlant<kNumInputs, kNumStates, kNumOutputs> plant_;
  Eigen::Matrix<double, kNumStates, kNumOutputs> L_;
};

}  // namespace control

}  // namespace muan

#include "state_space_observer.hpp"

#endif /* MUAN_CONTROL_STATE_SPACE_OBSERVER_H_ */
