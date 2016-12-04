#ifndef MUAN_CONTROL_STATE_SPACE_CONTROLLER_HPP_
#define MUAN_CONTROL_STATE_SPACE_CONTROLLER_HPP_

#include <numeric>
#include "muan/utils/math_utils.h"
#include "state_space_controller.h"

namespace muan {

namespace control {

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
StateSpaceController<kNumInputs, kNumStates, kNumOutputs>::StateSpaceController()
    : K_{Eigen::Matrix<double, kNumInputs, kNumStates>::Zero()},
      Kff_{Eigen::Matrix<double, kNumInputs, kNumStates>::Zero()},
      A_{Eigen::Matrix<double, kNumStates, kNumStates>::Identity()},
      r_{Eigen::Matrix<double, kNumStates, 1>::Zero()},
      u_min_{Eigen::Matrix<double, kNumInputs, 1>::Constant(-std::numeric_limits<double>::infinity())},
      u_max_{Eigen::Matrix<double, kNumInputs, 1>::Constant(std::numeric_limits<double>::infinity())} {}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
StateSpaceController<kNumInputs, kNumStates, kNumOutputs>::StateSpaceController(
    const Eigen::Matrix<double, kNumInputs, kNumStates>& K, const Eigen::Matrix<double, kNumInputs, 1>& u_min,
    const Eigen::Matrix<double, kNumInputs, 1>& u_max)
    : K_{K},
      Kff_{Eigen::Matrix<double, kNumInputs, kNumStates>::Zero()},
      A_{Eigen::Matrix<double, kNumStates, kNumStates>::Identity()},
      r_{Eigen::Matrix<double, kNumStates, 1>::Zero()},
      u_min_{u_min},
      u_max_{u_max} {}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
StateSpaceController<kNumInputs, kNumStates, kNumOutputs>::StateSpaceController(
    const Eigen::Matrix<double, kNumInputs, kNumStates>& K,
    const Eigen::Matrix<double, kNumInputs, kNumStates>& Kff,
    const Eigen::Matrix<double, kNumStates, kNumStates>& A, const Eigen::Matrix<double, kNumInputs, 1>& u_min,
    const Eigen::Matrix<double, kNumInputs, 1>& u_max)
    : K_{K},
      Kff_{Kff},
      A_{A},
      r_{Eigen::Matrix<double, kNumStates, 1>::Zero()},
      u_min_{u_min},
      u_max_{u_max} {}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
Eigen::Matrix<double, kNumInputs, 1> StateSpaceController<kNumInputs, kNumStates, kNumOutputs>::Update(
    const Eigen::Matrix<double, kNumStates, 1>& x) {
  Eigen::Matrix<double, kNumInputs, 1> u = K() * (r_ - x) + Kff() * (r_ - A() * r_);
  return CapMatrix(u, u_min(), u_max());
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
Eigen::Matrix<double, kNumInputs, 1> StateSpaceController<kNumInputs, kNumStates, kNumOutputs>::Update(
    const Eigen::Matrix<double, kNumStates, 1>& x, const Eigen::Matrix<double, kNumStates, 1>& r) {
  Eigen::Matrix<double, kNumInputs, 1> u = K() * (r_ - x) + Kff() * (r - A() * r_);
  r_ = r;
  return CapMatrix(u, u_min(), u_max());
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
const Eigen::Matrix<double, kNumStates, 1>& StateSpaceController<kNumInputs, kNumStates, kNumOutputs>::r()
    const {
  return r_;
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
Eigen::Matrix<double, kNumStates, 1>& StateSpaceController<kNumInputs, kNumStates, kNumOutputs>::r() {
  return r_;
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
double StateSpaceController<kNumInputs, kNumStates, kNumOutputs>::r(uint32_t i) const {
  return r_(i);
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
double& StateSpaceController<kNumInputs, kNumStates, kNumOutputs>::r(uint32_t i) {
  return r_(i);
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
const Eigen::Matrix<double, kNumInputs, 1>& StateSpaceController<kNumInputs, kNumStates, kNumOutputs>::u_min()
    const {
  return u_min_;
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
Eigen::Matrix<double, kNumInputs, 1>& StateSpaceController<kNumInputs, kNumStates, kNumOutputs>::u_min() {
  return u_min_;
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
double StateSpaceController<kNumInputs, kNumStates, kNumOutputs>::u_min(uint32_t i) const {
  return u_min_(i);
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
double& StateSpaceController<kNumInputs, kNumStates, kNumOutputs>::u_min(uint32_t i) {
  return u_min_(i);
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
const Eigen::Matrix<double, kNumInputs, 1>& StateSpaceController<kNumInputs, kNumStates, kNumOutputs>::u_max()
    const {
  return u_max_;
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
Eigen::Matrix<double, kNumInputs, 1>& StateSpaceController<kNumInputs, kNumStates, kNumOutputs>::u_max() {
  return u_max_;
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
double StateSpaceController<kNumInputs, kNumStates, kNumOutputs>::u_max(uint32_t i) const {
  return u_max_(i);
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
double& StateSpaceController<kNumInputs, kNumStates, kNumOutputs>::u_max(uint32_t i) {
  return u_max_(i);
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
const Eigen::Matrix<double, kNumInputs, kNumStates>&
StateSpaceController<kNumInputs, kNumStates, kNumOutputs>::K() const {
  return K_;
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
Eigen::Matrix<double, kNumInputs, kNumStates>&
StateSpaceController<kNumInputs, kNumStates, kNumOutputs>::K() {
  return K_;
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
double StateSpaceController<kNumInputs, kNumStates, kNumOutputs>::K(uint32_t i, uint32_t j) const {
  return K_(i, j);
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
double& StateSpaceController<kNumInputs, kNumStates, kNumOutputs>::K(uint32_t i, uint32_t j) {
  return K_(i, j);
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
const Eigen::Matrix<double, kNumInputs, kNumStates>&
StateSpaceController<kNumInputs, kNumStates, kNumOutputs>::Kff() const {
  return Kff_;
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
Eigen::Matrix<double, kNumInputs, kNumStates>&
StateSpaceController<kNumInputs, kNumStates, kNumOutputs>::Kff() {
  return Kff_;
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
double StateSpaceController<kNumInputs, kNumStates, kNumOutputs>::Kff(uint32_t i, uint32_t j) const {
  return Kff_(i, j);
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
double& StateSpaceController<kNumInputs, kNumStates, kNumOutputs>::Kff(uint32_t i, uint32_t j) {
  return Kff_(i, j);
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
const Eigen::Matrix<double, kNumStates, kNumStates>&
StateSpaceController<kNumInputs, kNumStates, kNumOutputs>::A() const {
  return A_;
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
Eigen::Matrix<double, kNumStates, kNumStates>&
StateSpaceController<kNumInputs, kNumStates, kNumOutputs>::A() {
  return A_;
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
double StateSpaceController<kNumInputs, kNumStates, kNumOutputs>::A(uint32_t i, uint32_t j) const {
  return A_(i, j);
}

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
double& StateSpaceController<kNumInputs, kNumStates, kNumOutputs>::A(uint32_t i, uint32_t j) {
  return A_(i, j);
}

}  // namespace control

}  // namespace muan

#endif /* MUAN_CONTROL_STATE_SPACE_CONTROLLER_HPP_ */
