#include "Eigen/Dense"
#include "state_space_controller.h"
#include "state_space_observer.h"
#include "state_space_plant.h"
#include "gtest/gtest.h"
#include <random>

TEST(StateSpace, Initialization) {
  muan::control::StateSpacePlant<1, 2, 1> plant;
  muan::control::StateSpaceController<1, 2, 1> controller;
  muan::control::StateSpaceObserver<1, 2, 1> observer;

  for (uint32_t i = 0; i < 2; i++) {
    for (uint32_t j = 0; j < 2; j++) {
      if (i == j) {
        EXPECT_EQ(plant.A(i, j), 1);
        EXPECT_EQ(controller.A(i, j), 1);
      } else {
        EXPECT_EQ(plant.A(i, j), 0);
        EXPECT_EQ(controller.A(i, j), 0);
      }
    }
    EXPECT_EQ(plant.x(i), 0);
    EXPECT_EQ(observer.x(i), 0);
    EXPECT_EQ(controller.K(0, i), 0);
    EXPECT_EQ(controller.Kff(0, i), 0);
    EXPECT_EQ(plant.B(i, 0), 0);
    EXPECT_EQ(plant.C(0, i), 0);
  }

  EXPECT_EQ(plant.D(0, 0), 0);
  EXPECT_EQ(controller.u_min(0), -std::numeric_limits<double>::infinity());
  EXPECT_EQ(controller.u_max(0), std::numeric_limits<double>::infinity());
}

TEST(StateSpace, StablePlant) {
  Eigen::Matrix<double, 2, 2> A;
  A << 1.0, .01, -.05, .95;
  muan::control::StateSpacePlant<1, 2, 1> plant(
      A, Eigen::Matrix<double, 2, 1>::Zero(),
      Eigen::Matrix<double, 1, 2>::Zero(), Eigen::Matrix<double, 1, 1>::Zero(),
      (Eigen::Matrix<double, 2, 1>() << 1.0, 1.0).finished());

  for (uint32_t i = 0; i < 100; i++) {
    plant.Update((Eigen::Matrix<double, 1, 1>() << 1).finished());
  }

  for (uint32_t i = 0; i < 2000; i++) {
    plant.Update((Eigen::Matrix<double, 1, 1>() << 0).finished());
  }

  for (uint32_t i = 0; i < 2; i++) {
    EXPECT_NEAR(plant.x(i), 0, 1e-6);
  }
}

TEST(StateSpace, UnstablePlant) {
  Eigen::Matrix<double, 2, 2> A;
  A << 1.0, .01, .05, 0.99;
  muan::control::StateSpacePlant<1, 2, 1> plant(
      A, Eigen::Matrix<double, 2, 1>::Zero(),
      Eigen::Matrix<double, 1, 2>::Zero(), Eigen::Matrix<double, 1, 1>::Zero(),
      (Eigen::Matrix<double, 2, 1>() << 1.0, 0.0).finished());

  for (uint32_t i = 0; i < 1000; i++) {
    plant.Update((Eigen::Matrix<double, 1, 1>() << 0).finished());
  }

  for (uint32_t i = 0; i < 2; i++) {
    EXPECT_GT(plant.x(i), 1e6);
  }
}

TEST(StateSpace, ControllerConverges) {
  muan::control::StateSpacePlant<1, 2, 1> plant;
  plant.A() << 1, 0.01, 0, 0.98;
  plant.B() << 1e-5, 0.02;
  plant.C() << 1, 0;
  plant.x() << 1, 0;

  muan::control::StateSpaceController<1, 2, 1> controller{};
  controller.K() << 10.0, 1.0;
  controller.A() = plant.A();
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

TEST(StateSpace, ControllerGoesToGoal) {
  muan::control::StateSpacePlant<1, 2, 1> plant;
  plant.A() << 1, 0.01, 0, 0.98;
  plant.B() << 1e-5, 0.02;
  plant.C() << 1, 0;
  plant.x() << 0, 0;

  muan::control::StateSpaceController<1, 2, 1> controller{};
  controller.K() << 10.0, 1.0;
  controller.A() = plant.A();
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

TEST(StateSpace, ControllerHoldsSteadyState) {
  muan::control::StateSpacePlant<1, 2, 1> plant;
  plant.A() << 1, 0.01, -.03, 0.98;
  plant.B() << 1e-5, 0.02;
  plant.C() << 1, 0;
  plant.x() << 0, 0;

  muan::control::StateSpaceController<1, 2, 1> controller{};
  controller.K() << 10.0, 1.0;
  controller.A() = plant.A();
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

TEST(StateSpace, ControllerObeysInputConstraints) {
  muan::control::StateSpacePlant<1, 2, 1> plant;
  plant.A() << 1, 0.01, 0, 0.98;
  plant.B() << 1e-5, 0.02;
  plant.C() << 1, 0;
  plant.x() << 0, 0;

  muan::control::StateSpaceController<1, 2, 1> controller{};
  controller.K() << 10.0, 1.0;
  controller.A() = plant.A();
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

TEST(StateSpace, ControllerTracksFeedForward) {
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

double GaussianNoise() {
  static std::mt19937_64 rng;
  static std::normal_distribution<double> dist;
  return dist(rng);
}

template <uint32_t A>
Eigen::Matrix<double, A, 1> NoiseVector(
    const Eigen::Matrix<double, A, A>& covariance) {
  Eigen::Matrix<double, A, 1> ret;

  for (uint32_t i = 0; i < A; i++) {
    ret[i] = GaussianNoise();
  }

  return covariance * ret;
}

TEST(StateSpace, ObserverRecoversFromNoise) {
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

  for (uint32_t t = 0; t < 1000; t++) {
    auto u = Eigen::Matrix<double, 1, 1>::Constant(1);
    observer.Update(u, plant.y() + NoiseVector<1>(R));
    plant.Update(u);
    plant.x() += NoiseVector<2>(Q);

    EXPECT_NEAR(plant.x(0), observer.x(0), 2e-2);
    EXPECT_NEAR(plant.x(1), observer.x(1), 2e-1);
  }

  EXPECT_NEAR(plant.x(0), observer.x(0), 1e-2);
  EXPECT_NEAR(plant.x(1), observer.x(1), 1e-1);
}
