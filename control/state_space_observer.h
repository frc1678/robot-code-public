#ifndef MUAN_CONTROL_STATE_SPACE_OBSERVER_H_
#define MUAN_CONTROL_STATE_SPACE_OBSERVER_H_

#include "Eigen/Core"
#include "state_space_plant.h"
#include <cstdint>

namespace muan {

namespace control {

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
class StateSpaceObserver {
 public:
  StateSpaceObserver();
  StateSpaceObserver(
      const muan::control::StateSpacePlant<kNumInputs, kNumStates, kNumOutputs>&
          plant,
      Eigen::Matrix<double, kNumStates, kNumOutputs> L =
          Eigen::Matrix<double, kNumStates, kNumOutputs>::Zero());
  virtual ~StateSpaceObserver();

  void Update(const Eigen::Matrix<double, kNumInputs, 1>& u,
              const Eigen::Matrix<double, kNumOutputs, 1>& y);

  const Eigen::Matrix<double, kNumStates, 1>& x() const;
  Eigen::Matrix<double, kNumStates, 1>& x();
  double x(uint32_t i) const;
  double& x(uint32_t i);

  const muan::control::StateSpacePlant<kNumInputs, kNumStates, kNumOutputs>&
  plant() const;
  muan::control::StateSpacePlant<kNumInputs, kNumStates, kNumOutputs>& plant();

  const Eigen::Matrix<double, kNumStates, kNumOutputs>& L() const;
  Eigen::Matrix<double, kNumStates, kNumOutputs>& L();
  double L(uint32_t i, uint32_t j) const;
  double& L(uint32_t i, uint32_t j);

 private:
  muan::control::StateSpacePlant<kNumInputs, kNumStates, kNumOutputs> plant_;
  Eigen::Matrix<double, kNumStates, kNumOutputs> L_;
};

} /* control */

} /* muan  */

#include "state_space_observer.hpp"

#endif /* MUAN_CONTROL_STATE_SPACE_OBSERVER_H_ */
