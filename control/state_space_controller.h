#ifndef MUAN_CONTROL_STATE_SPACE_CONTROLLER_H_
#define MUAN_CONTROL_STATE_SPACE_CONTROLLER_H_

#include "Eigen/Core"
#include <cstdint>

namespace muan {

namespace control {

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
class StateSpaceController {
 public:
  StateSpaceController();
  StateSpaceController(const Eigen::Matrix<double, kNumInputs, kNumStates>& K,
                       const Eigen::Matrix<double, kNumInputs, 1>& u_min =
                           Eigen::Matrix<double, kNumInputs, 1>::Constant(
                               -std::numeric_limits<double>::infinity()),
                       const Eigen::Matrix<double, kNumInputs, 1>& u_max =
                           Eigen::Matrix<double, kNumInputs, 1>::Constant(
                               std::numeric_limits<double>::infinity()));
  StateSpaceController(const Eigen::Matrix<double, kNumInputs, kNumStates>& K,
                       const Eigen::Matrix<double, kNumInputs, kNumStates>& Kff,
                       const Eigen::Matrix<double, kNumStates, kNumStates>& A,
                       const Eigen::Matrix<double, kNumInputs, 1>& u_min =
                           Eigen::Matrix<double, kNumInputs, 1>::Constant(
                               -std::numeric_limits<double>::infinity()),
                       const Eigen::Matrix<double, kNumInputs, 1>& u_max =
                           Eigen::Matrix<double, kNumInputs, 1>::Constant(
                               std::numeric_limits<double>::infinity()));
  virtual ~StateSpaceController();

  Eigen::Matrix<double, kNumInputs, 1> Update(
      const Eigen::Matrix<double, kNumStates, 1>& x,
      const Eigen::Matrix<double, kNumStates, 1>& r);
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

} /* control */

} /* muan */

#include "state_space_controller.hpp"

#endif /* MUAN_CONTROL_STATE_SPACE_CONTROLLER_H_ */
