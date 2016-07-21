#ifndef MUAN_CONTROL_STATE_SPACE_PLANT_H_
#define MUAN_CONTROL_STATE_SPACE_PLANT_H_

#include "Eigen/Core"
#include <cstdint>

namespace muan {

namespace control {

template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
class StateSpacePlant {
 public:
  StateSpacePlant();
  StateSpacePlant(const Eigen::Matrix<double, kNumStates, kNumStates>& A,
                  const Eigen::Matrix<double, kNumStates, kNumInputs>& B,
                  const Eigen::Matrix<double, kNumOutputs, kNumStates>& C,
                  const Eigen::Matrix<double, kNumOutputs, kNumInputs>& D =
                      Eigen::Matrix<double, kNumOutputs, kNumInputs>::Zeros(),
                  const Eigen::Matrix<double, kNumStates, 1>& x_0 =
                      Eigen::Matrix<double, kNumStates, 1>::Zeros());
  virtual ~StateSpacePlant();

  void Update(const Eigen::Matrix<double, kNumInputs, 1>& u);

  const Eigen::Matrix<double, kNumStates, 1>& x() const;
  Eigen::Matrix<double, kNumStates, 1>& x();
  double x(uint32_t i) const;
  double& x(uint32_t i);

  Eigen::Matrix<double, kNumOutputs, 1> y() const;
  double y(uint32_t i) const;

  const Eigen::Matrix<double, kNumStates, kNumStates>& A() const;
  Eigen::Matrix<double, kNumStates, kNumStates>& A();
  double A(uint32_t i, uint32_t j) const;
  double& A(uint32_t i, uint32_t j);

  const Eigen::Matrix<double, kNumStates, kNumInputs>& B() const;
  Eigen::Matrix<double, kNumStates, kNumInputs>& B();
  double B(uint32_t i, uint32_t j) const;
  double& B(uint32_t i, uint32_t j);

  const Eigen::Matrix<double, kNumOutputs, kNumStates>& C() const;
  Eigen::Matrix<double, kNumOutputs, kNumStates>& C();
  double C(uint32_t i, uint32_t j) const;
  double& C(uint32_t i, uint32_t j);

  const Eigen::Matrix<double, kNumOutputs, kNumInputs>& D() const;
  Eigen::Matrix<double, kNumOutputs, kNumInputs>& D();
  double D(uint32_t i, uint32_t j) const;
  double& D(uint32_t i, uint32_t j);

 private:
  Eigen::Matrix<double, kNumStates, kNumStates> A_;
  Eigen::Matrix<double, kNumStates, kNumInputs> B_;
  Eigen::Matrix<double, kNumOutputs, kNumStates> C_;
  Eigen::Matrix<double, kNumOutputs, kNumInputs> D_;

  Eigen::Matrix<double, kNumStates, 1> x_;
};

} /* control */

} /* muan */

#include "state_space_plant.hpp"

#endif /* MUAN_CONTROL_STATE_SPACE_PLANT_H_ */
