#ifndef MUAN_CONTROL_STATE_SPACE_PLANT_HPP_
#define MUAN_CONTROL_STATE_SPACE_PLANT_HPP_

#include "state_space_plant.h"

namespace muan {

namespace control {

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
StateSpacePlant<kNumInputs, kNumStates, kNumOutputs>::StateSpacePlant()
    : A_{Eigen::Matrix<double, kNumStates, kNumStates>::Identity()},
      B_{Eigen::Matrix<double, kNumStates, kNumInputs>::Zero()},
      C_{Eigen::Matrix<double, kNumOutputs, kNumStates>::Zero()},
      D_{Eigen::Matrix<double, kNumOutputs, kNumInputs>::Zero()},
      x_{Eigen::Matrix<double, kNumStates, 1>::Zero()} {}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
StateSpacePlant<kNumInputs, kNumStates, kNumOutputs>::StateSpacePlant(
    const Eigen::Matrix<double, kNumStates, kNumStates>& A,
    const Eigen::Matrix<double, kNumStates, kNumInputs>& B,
    const Eigen::Matrix<double, kNumOutputs, kNumStates>& C,
    const Eigen::Matrix<double, kNumOutputs, kNumInputs>& D,
    const Eigen::Matrix<double, kNumStates, 1>& x_0)
    : A_{A}, B_{B}, C_{C}, D_{D}, x_{x_0} {}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
StateSpacePlant<kNumInputs, kNumStates, kNumOutputs>::~StateSpacePlant() {}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
void StateSpacePlant<kNumInputs, kNumStates, kNumOutputs>::Update(
    const Eigen::Matrix<double, kNumInputs, 1>& u) {
  x_ = A_ * x_ + B_ * u;
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
const Eigen::Matrix<double, kNumStates, 1>&
StateSpacePlant<kNumInputs, kNumStates, kNumOutputs>::x() const {
  return x_;
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
Eigen::Matrix<double, kNumStates, 1>&
StateSpacePlant<kNumInputs, kNumStates, kNumOutputs>::x() {
  return x_;
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
double StateSpacePlant<kNumInputs, kNumStates, kNumOutputs>::x(
    uint32_t i) const {
  return x_(i);
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
double& StateSpacePlant<kNumInputs, kNumStates, kNumOutputs>::x(uint32_t i) {
  return x_(i);
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
Eigen::Matrix<double, kNumOutputs, 1>
StateSpacePlant<kNumInputs, kNumStates, kNumOutputs>::y() const {
  return C_ * x_;
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
double StateSpacePlant<kNumInputs, kNumStates, kNumOutputs>::y(
    uint32_t i) const {
  return y()(i);
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
const Eigen::Matrix<double, kNumStates, kNumStates>&
StateSpacePlant<kNumInputs, kNumStates, kNumOutputs>::A() const {
  return A_;
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
Eigen::Matrix<double, kNumStates, kNumStates>&
StateSpacePlant<kNumInputs, kNumStates, kNumOutputs>::A() {
  return A_;
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
double StateSpacePlant<kNumInputs, kNumStates, kNumOutputs>::A(
    uint32_t i, uint32_t j) const {
  return A_(i, j);
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
double& StateSpacePlant<kNumInputs, kNumStates, kNumOutputs>::A(uint32_t i,
                                                                uint32_t j) {
  return A_(i, j);
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
const Eigen::Matrix<double, kNumStates, kNumInputs>&
StateSpacePlant<kNumInputs, kNumStates, kNumOutputs>::B() const {
  return B_;
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
Eigen::Matrix<double, kNumStates, kNumInputs>&
StateSpacePlant<kNumInputs, kNumStates, kNumOutputs>::B() {
  return B_;
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
double StateSpacePlant<kNumInputs, kNumStates, kNumOutputs>::B(
    uint32_t i, uint32_t j) const {
  return B_(i, j);
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
double& StateSpacePlant<kNumInputs, kNumStates, kNumOutputs>::B(uint32_t i,
                                                                uint32_t j) {
  return B_(i, j);
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
const Eigen::Matrix<double, kNumOutputs, kNumStates>&
StateSpacePlant<kNumInputs, kNumStates, kNumOutputs>::C() const {
  return C_;
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
Eigen::Matrix<double, kNumOutputs, kNumStates>&
StateSpacePlant<kNumInputs, kNumStates, kNumOutputs>::C() {
  return C_;
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
double StateSpacePlant<kNumInputs, kNumStates, kNumOutputs>::C(
    uint32_t i, uint32_t j) const {
  return C_(i, j);
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
double& StateSpacePlant<kNumInputs, kNumStates, kNumOutputs>::C(uint32_t i,
                                                                uint32_t j) {
  return C_(i, j);
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
const Eigen::Matrix<double, kNumOutputs, kNumInputs>&
StateSpacePlant<kNumInputs, kNumStates, kNumOutputs>::D() const {
  return D_;
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
Eigen::Matrix<double, kNumOutputs, kNumInputs>&
StateSpacePlant<kNumInputs, kNumStates, kNumOutputs>::D() {
  return D_;
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
double StateSpacePlant<kNumInputs, kNumStates, kNumOutputs>::D(
    uint32_t i, uint32_t j) const {
  return D_(i, j);
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
double& StateSpacePlant<kNumInputs, kNumStates, kNumOutputs>::D(uint32_t i,
                                                                uint32_t j) {
  return D_(i, j);
}

} /* control */

} /* muan  */

#endif /* MUAN_CONTROL_STATE_SPACE_PLANT_HPP_ */
