#include <limits>
#include <random>
#include "Eigen/Dense"
#include "gtest/gtest.h"
#include "muan/control/state_space_controller.h"
#include "muan/control/state_space_observer.h"
#include "muan/control/state_space_plant.h"
#include "muan/utils/math_utils.h"

// Ensure that all state-space objects are properly zero-initialized
TEST(StateSpace, Initialization) {
  muan::control::StateSpacePlant<1, 2, 1> plant;
  muan::control::StateSpaceController<1, 2, 1> controller;
  muan::control::StateSpaceObserver<1, 2, 1> observer;

  for (uint32_t i = 0; i < 2; i++) {
    // A should be initialized to the identity matrix, for a static system by
    // default
    for (uint32_t j = 0; j < 2; j++) {
      if (i == j) {
        EXPECT_EQ(plant.A(i, j), 1);
        EXPECT_EQ(controller.A(i, j), 1);
      } else {
        EXPECT_EQ(plant.A(i, j), 0);
        EXPECT_EQ(controller.A(i, j), 0);
      }
    }
    // Everything else should be initialized to zero
    EXPECT_EQ(plant.x(i), 0);
    EXPECT_EQ(observer.x(i), 0);
    EXPECT_EQ(controller.K(0, i), 0);
    EXPECT_EQ(controller.Kff(0, i), 0);
    EXPECT_EQ(plant.B(i, 0), 0);
    EXPECT_EQ(plant.C(0, i), 0);
  }

  EXPECT_EQ(plant.D(0, 0), 0);

  // The control signal boundaries should be initialized to +/- infinity by
  // default
  EXPECT_EQ(controller.u_min(0), -std::numeric_limits<double>::infinity());
  EXPECT_EQ(controller.u_max(0), std::numeric_limits<double>::infinity());

  // Set matrix entries
  for (uint32_t i = 0; i < 2; i++) {
    for (uint32_t j = 0; j < 2; j++) {
      plant.A(i, j) = 2 * i + j;
    }
    plant.B(i, 0) = i + 4;
    plant.C(0, i) = i + 6;
  }

  // Use default parameters
  muan::control::StateSpacePlant<1, 2, 1> plant2(plant.A(), plant.B(),
                                                 plant.C());

  // Ensure that the plant uses the gains given to it
  for (uint32_t i = 0; i < 2; i++) {
    for (uint32_t j = 0; j < 2; j++) {
      EXPECT_EQ(plant2.A(i, j), plant.A(i, j));
    }
    EXPECT_EQ(plant2.B(i, 0), plant.B(i, 0));
    EXPECT_EQ(plant2.C(0, i), plant.C(0, i));
  }
}

// Ensure that a mathematically stable plant converges to zero
TEST(StateSpace, StablePlant) {
  // Discrete-time open-loop poles are 0.986 and 0.964
  Eigen::Matrix<double, 2, 2> A;
  A << 1.0, .01, -.05, .95;

  muan::control::StateSpacePlant<1, 2, 1> plant(
      A, Eigen::Matrix<double, 2, 1>::Zero(),
      Eigen::Matrix<double, 1, 2>::Zero(), Eigen::Matrix<double, 1, 1>::Zero(),
      (Eigen::Matrix<double, 2, 1>() << 1.0, 1.0).finished());

  auto u = Eigen::Matrix<double, 1, 1>::Zero();

  for (uint32_t i = 0; i < 100; i++) {
    plant.Update(u);
  }

  for (uint32_t i = 0; i < 2000; i++) {
    plant.Update(u);
  }

  for (uint32_t i = 0; i < 2; i++) {
    EXPECT_NEAR(plant.x(i), 0, 1e-6);
  }
}

// Ensure that a mathmatically unstable plant diverges to infinity
TEST(StateSpace, UnstablePlant) {
  // Discrete-time open-loop poles are 1.018 and 0.972
  Eigen::Matrix<double, 2, 2> A;
  A << 1.0, .01, .05, 0.99;

  muan::control::StateSpacePlant<1, 2, 1> plant(
      A, Eigen::Matrix<double, 2, 1>::Zero(),
      Eigen::Matrix<double, 1, 2>::Zero(), Eigen::Matrix<double, 1, 1>::Zero(),
      (Eigen::Matrix<double, 2, 1>() << 1.0, 0.0).finished());

  auto u = Eigen::Matrix<double, 1, 1>::Zero();

  for (uint32_t i = 0; i < 1000; i++) {
    plant.Update(u);
  }

  for (uint32_t i = 0; i < 2; i++) {
    EXPECT_GT(plant.x(i), 1e6);
  }
}

// Ensure that adding a controller causes a plant to converge to zero
TEST(StateSpace, ControllerConverges) {
  // Discrete-time closed-loop poles are 0.987 and 0.952
  muan::control::StateSpacePlant<1, 2, 1> plant;
  plant.A() << 1, 0.01, 0, 0.98;
  plant.B() << 1e-5, 0.02;
  plant.C() << 1, 0;
  plant.x() << 1, 0;

  muan::control::StateSpaceController<1, 2, 1> controller{};
  controller.K() << 10.0, 1.0;
  controller.A() = plant.A();
  // Kff is the pseudoinverse of B
  controller.Kff() =
      (plant.B().transpose() * plant.B()).inverse() * plant.B().transpose();
  controller.r() << 0.0, 0.0;

  for (uint32_t t = 0; t < 1000; t++) {
    auto u = controller.Update(plant.x());
    plant.Update(u);
  }

  EXPECT_NEAR(plant.x(0), controller.r(0), 1e-6);
  EXPECT_NEAR(plant.x(1), controller.r(1), 1e-6);
}

// Ensure that a controller will converge to a goal if supplied
TEST(StateSpace, ControllerGoesToGoal) {
  // Discrete-time closed-loop poles are 0.987 and 0.952
  muan::control::StateSpacePlant<1, 2, 1> plant;
  plant.A() << 1, 0.01, 0, 0.98;
  plant.B() << 1e-5, 0.02;
  plant.C() << 1, 0;
  plant.x() << 0, 0;

  muan::control::StateSpaceController<1, 2, 1> controller{};
  controller.K() << 10.0, 1.0;
  controller.A() = plant.A();
  // Kff is the pseudoinverse of B
  controller.Kff() =
      (plant.B().transpose() * plant.B()).inverse() * plant.B().transpose();
  controller.r() << 1.0, 0.0;

  for (uint32_t t = 0; t < 1000; t++) {
    auto u = controller.Update(plant.x());
    plant.Update(u);
  }

  EXPECT_NEAR(plant.x(0), controller.r(0), 1e-6);
  EXPECT_NEAR(plant.x(1), controller.r(1), 1e-6);
}

// Ensure the controller will hold a correct steady-state goal even if the goal
// requires a nonzero control signal (via feedforward)
TEST(StateSpace, ControllerHoldsSteadyState) {
  // Discrete-time closed-loop poles are 0.995 and 0.945
  muan::control::StateSpacePlant<1, 2, 1> plant;
  plant.A() << 1, 0.01, -.03, 0.98;
  plant.B() << 1e-5, 0.02;
  plant.C() << 1, 0;
  plant.x() << 0, 0;

  muan::control::StateSpaceController<1, 2, 1> controller{};
  controller.K() << 10.0, 1.0;
  controller.A() = plant.A();
  // Kff is the pseudoinverse of B
  controller.Kff() =
      (plant.B().transpose() * plant.B()).inverse() * plant.B().transpose();
  controller.r() << 1.0, 0.0;

  for (uint32_t t = 0; t < 1000; t++) {
    auto u = controller.Update(plant.x());
    plant.Update(u);
  }

  EXPECT_NEAR(plant.x(0), controller.r(0), 1e-2);
  EXPECT_NEAR(plant.x(1), controller.r(1), 1e-2);
}

// Ensure that the control signal from the controller stays within the specified
// bounds
TEST(StateSpace, ControllerObeysInputConstraints) {
  // Discrete-time closed-loop poles are 0.987 and 0.952
  muan::control::StateSpacePlant<1, 2, 1> plant;
  plant.A() << 1, 0.01, 0, 0.98;
  plant.B() << 1e-5, 0.02;
  plant.C() << 1, 0;
  plant.x() << 0, 0;

  muan::control::StateSpaceController<1, 2, 1> controller{};
  controller.K() << 10.0, 1.0;
  controller.A() = plant.A();
  // Kff is the pseudoinverse of B
  controller.Kff() =
      (plant.B().transpose() * plant.B()).inverse() * plant.B().transpose();
  controller.r() << 1.0, 0.0;
  controller.u_max() << 12;
  controller.u_min() << -12;

  for (uint32_t t = 0; t < 1000; t++) {
    auto u = controller.Update(plant.x());
    EXPECT_LT(u(0), controller.u_max(0));
    EXPECT_GT(u(0), controller.u_min(0));
    plant.Update(u);
  }

  EXPECT_NEAR(plant.x(0), controller.r(0), 1e-6);
  EXPECT_NEAR(plant.x(1), controller.r(1), 1e-6);
}

// Ensure that the controller correctly tracks a moving goal
TEST(StateSpace, ControllerTracksFeedForward) {
  // System discretized from continuous-time system, so it should be dynamically
  // consistent
  muan::control::StateSpacePlant<1, 2, 1> plant;
  plant.A() << 1, 9.9502e-3, 0, 9.9005e-1;
  plant.B() << 4.9834e-5, 9.9502e-3;
  plant.C() << 1, 0;
  plant.x() << 0, 0;

  muan::control::StateSpaceController<1, 2, 1> controller{};
  // Use a zero feedback matrix so we're only relying on feedforwards control
  controller.K() << 0, 0;
  controller.A() = plant.A();
  // Kff is the pseudoinverse of B
  controller.Kff() =
      (plant.B().transpose() * plant.B()).inverse() * plant.B().transpose();
  controller.r() << 0, 0;

  Eigen::Matrix<double, 2, 1> r = controller.r();
  Eigen::Matrix<double, 1, 1> expected_u;
  expected_u << 1;

  for (uint32_t t = 0; t < 1000; t++) {
    r = plant.A() * r + plant.B() * expected_u;

    auto u = controller.Update(plant.x(), r);
    plant.Update(u);
    EXPECT_NEAR(plant.x(0), controller.r(0), 1e-3);
    EXPECT_NEAR(plant.x(1), controller.r(1), 1e-3);

    EXPECT_NEAR(u(0), expected_u(0), 1e-3);
  }

  EXPECT_NEAR(plant.x(0), controller.r(0), 1e-3);
  EXPECT_NEAR(plant.x(1), controller.r(1), 1e-3);
}

// Ensure that the observer's estimation converges to the correct state after a
// large initial error in a static system
TEST(StateSpace, ObserverRecoversFromInitialError) {
  muan::control::StateSpacePlant<1, 2, 1> plant;
  plant.A() << 1, 9.9502e-3, 0, 9.9005e-1;
  plant.B() << 4.9834e-5, 9.9502e-3;
  plant.C() << 1, 0;
  plant.x() << 0, 0;

  muan::control::StateSpaceController<1, 2, 1> controller{};
  controller.K() << 0, 0;
  controller.A() = plant.A();
  controller.Kff() =
      (plant.B().transpose() * plant.B()).inverse() * plant.B().transpose();
  controller.r() << 0, 0;

  muan::control::StateSpaceObserver<1, 2, 1> observer{plant};
  observer.L() << 1e-1, 1;

  plant.x(0) = 1;

  for (uint32_t t = 0; t < 1000; t++) {
    auto u = Eigen::Matrix<double, 1, 1>::Zero();
    plant.Update(u);
    observer.Update(u, plant.y());
  }

  EXPECT_NEAR(plant.x(0), observer.x(0), 1e-6);
  EXPECT_NEAR(plant.x(1), observer.x(1), 1e-6);
}

// Ensure that the observer's estimation converges to the correct state after a
// large initial error in a moving system
TEST(StateSpace, ObserverRecoversFromInitialErrorMovingSystem) {
  muan::control::StateSpacePlant<1, 2, 1> plant;
  plant.A() << 1, 9.9502e-3, 0, 9.9005e-1;
  plant.B() << 4.9834e-5, 9.9502e-3;
  plant.C() << 1, 0;
  plant.x() << 0, 0;

  muan::control::StateSpaceController<1, 2, 1> controller{};
  controller.K() << 0, 0;
  controller.A() = plant.A();
  controller.Kff() =
      (plant.B().transpose() * plant.B()).inverse() * plant.B().transpose();
  controller.r() << 0, 0;

  muan::control::StateSpaceObserver<1, 2, 1> observer{plant};
  observer.L() << 1e-1, 1;

  plant.x(0) = 1;

  for (uint32_t t = 0; t < 1000; t++) {
    auto u = Eigen::Matrix<double, 1, 1>::Constant(1);
    observer.Update(u, plant.y());
    plant.Update(u);
  }

  EXPECT_NEAR(plant.x(0), observer.x(0), 1e-6);
  EXPECT_NEAR(plant.x(1), observer.x(1), 1e-6);
}

// Ensure that the observer's estimation converges to the correct state when its
// model is slightly incorrect
TEST(StateSpace, ObserverRecoversFromIncorrectModel) {
  muan::control::StateSpacePlant<1, 2, 1> plant;
  plant.A() << 1, 9.9502e-3, 0, 9.9005e-1;
  plant.B() << 4.9834e-5, 9.9502e-3;
  plant.C() << 1, 0;
  plant.x() << 0, 0;

  muan::control::StateSpaceController<1, 2, 1> controller{};
  controller.K() << 0, 0;
  controller.A() = plant.A();
  controller.Kff() =
      (plant.B().transpose() * plant.B()).inverse() * plant.B().transpose();
  controller.r() << 0, 0;

  muan::control::StateSpaceObserver<1, 2, 1> observer{plant};
  observer.L() << 2e-1, 3;

  plant.A(1, 1) *= .985;
  plant.A(0, 1) *= .995;

  for (uint32_t t = 0; t < 1000; t++) {
    auto u = Eigen::Matrix<double, 1, 1>::Constant(1);
    observer.Update(u, plant.y());
    plant.Update(u);
  }

  EXPECT_NEAR(plant.x(0), observer.x(0), 1e-2);
  EXPECT_NEAR(plant.x(1), observer.x(1), 1e-1);
}

// Ensure that the observer is able to compensate for a noisy signal
TEST(StateSpace, ObserverRecoversFromNoise) {
  // Discrete-time Q matrix and continuous-time R matrix
  Eigen::Matrix<double, 2, 2> Q;
  Q << .001, 0, 0, .001;
  Eigen::Matrix<double, 1, 1> R;
  R << .003;

  muan::control::StateSpacePlant<1, 2, 1> plant;
  plant.A() << 1, 9.9502e-3, 0, 9.9005e-1;
  plant.B() << 4.9834e-5, 9.9502e-3;
  plant.C() << 1, 0;
  plant.x() << 0, 0;

  muan::control::StateSpaceController<1, 2, 1> controller{};
  controller.K() << 0, 0;
  controller.A() = plant.A();
  controller.Kff() =
      (plant.B().transpose() * plant.B()).inverse() * plant.B().transpose();
  controller.r() << 0, 0;

  muan::control::StateSpaceObserver<1, 2, 1> observer{plant};
  observer.L() << 2e-1, 5;

  // Apply both process noise (Q) and measurement noise (R) to the simulation
  for (uint32_t t = 0; t < 1000; t++) {
    auto u = Eigen::Matrix<double, 1, 1>::Constant(1);
    observer.Update(u, plant.y() + muan::utils::GaussianNoise<1>(R));
    plant.Update(u);
    plant.x() += muan::utils::GaussianNoise<2>(Q);

    EXPECT_NEAR(plant.x(0), observer.x(0), 2e-2);
    EXPECT_NEAR(plant.x(1), observer.x(1), 2e-1);
  }

  EXPECT_NEAR(plant.x(0), observer.x(0), 1e-2);
  EXPECT_NEAR(plant.x(1), observer.x(1), 1e-1);
}
