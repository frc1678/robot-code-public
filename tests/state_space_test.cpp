#include "gtest/gtest.h"
#include "../control/state_space_plant.h"
#include "../control/state_feedback_controller.h"

using namespace muan;

TEST(StateSpacePlantTest, StableSystem) {
  Eigen::Matrix<double, 2, 2> a;
  a << 1, .05, 0, 0.97;
  Eigen::Matrix<double, 2, 1> b;
  b << 0, .05;
  Eigen::Matrix<double, 1, 2> c;
  c << 1, 0;
  StateSpacePlant<1, 2, 1, true> flywheel_plant_(a, b, c);
  for (int i = 0; i < 100; i++) {
    Eigen::Matrix<double, 1, 1> u;
    u << 1.0;
    flywheel_plant_.Update(u);
  }
  for (int i = 0; i < 1000; i++) {
    Eigen::Matrix<double, 1, 1> u;
    u << 0.0;
    flywheel_plant_.Update(u);
  }
  ASSERT_NEAR(flywheel_plant_.GetX()(1), 0, 1e-10);
}

TEST(StateSpacePlantTest, UnstableSystem) {
  Eigen::Matrix<double, 2, 2> a;
  a << 1, .05, 0, 1.03;
  Eigen::Matrix<double, 2, 1> b;
  b << 0, .05;
  Eigen::Matrix<double, 1, 2> c;
  c << 1, 0;
  StateSpacePlant<1, 2, 1, true> flywheel_plant_(a, b, c);
  for (int i = 0; i < 100; i++) {
    Eigen::Matrix<double, 1, 1> u;
    u << 1.0;
    flywheel_plant_.Update(u);
  }
  for (int i = 0; i < 1000; i++) {
    Eigen::Matrix<double, 1, 1> u;
    u << 0.0;
    flywheel_plant_.Update(u);
  }
  ASSERT_GT(flywheel_plant_.GetX()(1), 1e10);
}

TEST(StateSpacePlantTest, SteadyState) {
  Eigen::Matrix<double, 2, 2> a;
  a << 1, .05, 0, 0.97;
  Eigen::Matrix<double, 2, 1> b;
  b << 0, .05;
  Eigen::Matrix<double, 1, 2> c;
  c << 1, 0;
  StateSpacePlant<1, 2, 1, true> flywheel_plant_(a, b, c);
  for (int i = 0; i < 1000; i++) {
    Eigen::Matrix<double, 1, 1> u;
    u << 1.0;
    flywheel_plant_.Update(u);
  }
  ASSERT_NEAR(flywheel_plant_.GetX()(1), 5.0 / 3.0, 1e-10);
}

TEST(StateSpaceControllerTest, GoesToZero) {
  Eigen::Matrix<double, 2, 2> a;
  a << 1, .05, 0, 0.9;
  Eigen::Matrix<double, 2, 1> b;
  b << 0, .05;
  Eigen::Matrix<double, 1, 2> c;
  c << 1, 0;
  StateSpacePlant<1, 2, 1, true> plant(a, b, c);
  Eigen::Matrix<double, 1, 2> k;
  k << -1.0, -1.0;
  StateFeedbackController<2, 1> controller(k);

  Eigen::Matrix<double, 2, 1> x0;
  x0 << 1.0, 3.0;
  plant.SetX(x0);
  for (int i = 0; i < 1000; i++) {
    auto u = -controller.Calculate(plant.GetX());
    plant.Update(u);
  }
  ASSERT_NEAR(plant.GetX()(0), 0.0, 1e-5);
  ASSERT_NEAR(plant.GetX()(1), 0.0, 1e-5);
}

TEST(StateSpaceControllerTest, GoesToGoal) {
  Eigen::Matrix<double, 2, 2> a;
  a << 1, .05, 0, 0.9;
  Eigen::Matrix<double, 2, 1> b;
  b << 0, .05;
  Eigen::Matrix<double, 1, 2> c;
  c << 1, 0;
  StateSpacePlant<1, 2, 1, true> plant(a, b, c);
  Eigen::Matrix<double, 1, 2> k;
  k << -1.0, -1.0;
  StateFeedbackController<2, 1> controller(k);

  Eigen::Matrix<double, 2, 1> x0;
  x0 << 1.0, 3.0;
  plant.SetX(x0);

  Eigen::Matrix<double, 2, 1> r;
  r << 1.0, 0.0;
  controller.SetGoal(r);

  for (int i = 0; i < 1000; i++) {
    auto u = -controller.Calculate(plant.GetX());
    plant.Update(u);
  }
  ASSERT_NEAR(plant.GetX()(0), r(0), 1e-5);
  ASSERT_NEAR(plant.GetX()(1), r(1), 1e-5);
}

TEST(DiscretizeSystem, Compiles) {
  Eigen::Matrix<double, 2, 2> a;
  a << 1.0, 0.0, 0.0, -1.0;
  Eigen::Matrix<double, 2, 1> b;
  b << 0, 1;
  Eigen::Matrix<double, 1, 2> c;
  c << 1, 0;
  auto plant = c2d(StateSpacePlant<1, 2, 1>(a, b, c), .005*s);
}
