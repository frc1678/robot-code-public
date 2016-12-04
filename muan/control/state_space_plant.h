#ifndef MUAN_CONTROL_STATE_SPACE_PLANT_H_
#define MUAN_CONTROL_STATE_SPACE_PLANT_H_

#include "Eigen/Core"
#include <cstdint>

namespace muan {

namespace control {

/*
 * A discrete system model in state-space form using predefined system matrices.
 * Mostly useful for testing purposes, but also used as a helper for
 * StateSpaceObserver.
 * Example:
 *    StateSpacePlant<1, 2, 1> plant{ SystemConstants::A,
 *                                    SystemConstants::B,
 *                                    SystemConstants::C,
 *                                    SystemConstants::D,
 *                                    initial_state };
 *    ...
 *    for (Time t = 0; t < final_time; t += dt) {
 *      ...
 *      plant.Update(simulated_u);
 *    }
 *    EXPECT_EQ(plant.x(0), expected_position);
 */
template <uint32_t kNumInputs, uint32_t kNumStates, uint32_t kNumOutputs>
class StateSpacePlant {
 public:
  // Initializes the system to sane defaults. A is set to the identity matrix
  // and all other matrices are set to zero, meaning that the system's position
  // will remain constant. Using this constructor implies that the matrices will
  // be set at a later time.
  StateSpacePlant();

  // Initializes the system from predefined discrete-time system matrices
  StateSpacePlant(
      const Eigen::Matrix<double, kNumStates, kNumStates>& A,
      const Eigen::Matrix<double, kNumStates, kNumInputs>& B,
      const Eigen::Matrix<double, kNumOutputs, kNumStates>& C,
      const Eigen::Matrix<double, kNumOutputs, kNumInputs>& D =
          Eigen::Matrix<double, kNumOutputs, kNumInputs>::Zero(),
      const Eigen::Matrix<double, kNumStates, 1>& x_0 = Eigen::Matrix<double, kNumStates, 1>::Zero());
  virtual ~StateSpacePlant() = default;

  // Updates the system by one timestep with a specified control signal and
  // difference equation:
  // x(n+1) = A*x(n) + B*u(n)
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

}  // namespace control

}  // namespace muan

#include "state_space_plant.hpp"

#endif /* MUAN_CONTROL_STATE_SPACE_PLANT_H_ */
