#ifndef MUAN_CONTROL_STATE_SPACE_OBSERVER_HPP_
#define MUAN_CONTROL_STATE_SPACE_OBSERVER_HPP_

#include "state_space_observer.h"

namespace muan {

namespace control {

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
StateSpaceObserver<kNumInputs, kNumStates, kNumOutputs>::StateSpaceObserver()
    : plant_{}, L_{Eigen::Matrix<double, kNumStates, kNumOutputs>::Zero()} {}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
StateSpaceObserver<kNumInputs, kNumStates, kNumOutputs>::StateSpaceObserver(
    const muan::control::StateSpacePlant<kNumInputs, kNumStates, kNumOutputs>&
        plant,
    Eigen::Matrix<double, kNumStates, kNumOutputs> L)
    : plant_{plant}, L_{L} {}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
void StateSpaceObserver<kNumInputs, kNumStates, kNumOutputs>::Update(
    const Eigen::Matrix<double, kNumInputs, 1>& u,
    const Eigen::Matrix<double, kNumOutputs, 1>& y) {
  plant().x() += L() * (y - plant().y());
  plant().Update(u);
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
const Eigen::Matrix<double, kNumStates, 1>&
StateSpaceObserver<kNumInputs, kNumStates, kNumOutputs>::x() const {
  return plant().x();
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
Eigen::Matrix<double, kNumStates, 1>&
StateSpaceObserver<kNumInputs, kNumStates, kNumOutputs>::x() {
  return plant().x();
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
double StateSpaceObserver<kNumInputs, kNumStates, kNumOutputs>::x(
    uint32_t i) const {
  return plant().x(i);
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
double& StateSpaceObserver<kNumInputs, kNumStates, kNumOutputs>::x(uint32_t i) {
  return plant().x(i);
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
const muan::control::StateSpacePlant<kNumInputs, kNumStates, kNumOutputs>&
StateSpaceObserver<kNumInputs, kNumStates, kNumOutputs>::plant() const {
  return plant_;
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
muan::control::StateSpacePlant<kNumInputs, kNumStates, kNumOutputs>&
StateSpaceObserver<kNumInputs, kNumStates, kNumOutputs>::plant() {
  return plant_;
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
const Eigen::Matrix<double, kNumStates, kNumOutputs>&
StateSpaceObserver<kNumInputs, kNumStates, kNumOutputs>::L() const {
  return L_;
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
Eigen::Matrix<double, kNumStates, kNumOutputs>&
StateSpaceObserver<kNumInputs, kNumStates, kNumOutputs>::L() {
  return L_;
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
double StateSpaceObserver<kNumInputs, kNumStates, kNumOutputs>::L(
    uint32_t i, uint32_t j) const {
  return L_(i, j);
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
double& StateSpaceObserver<kNumInputs, kNumStates, kNumOutputs>::L(uint32_t i,
                                                                   uint32_t j) {
  return L_(i, j);
}

}  // namespace control

}  // namespace muan

#endif /* MUAN_CONTROL_STATE_SPACE_OBSERVER_HPP_ */
