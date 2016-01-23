#include "gtest/gtest.h"
#include "Eigen/Core"
#include "../control/state_space_plant.h"
#include "../control/control_utils.h"
#include "../control/state_feedback_controller.h"

using namespace muan;

TEST(StateSpacePlantTest, StableSystem) {
  auto a = as_matrix<2, 2>({{1, .05}, {0, .97}});
  auto b = as_matrix<2, 1>({{0}, {.05}});
  auto c = as_matrix<1, 2>({{1, 0}});
  StateSpacePlant<1, 2, 1, true> flywheel_plant_(a, b, c);
  for (int i = 0; i < 100; i++) {
    auto u = as_matrix<1, 1>({{1}});
    flywheel_plant_.Update(u);
  }
  for (int i = 0; i < 1000; i++) {
    auto u = as_matrix<1, 1>({{0.0}});
    flywheel_plant_.Update(u);
  }
  ASSERT_NEAR(flywheel_plant_.GetX()(1), 0, 1e-10);
}

TEST(StateSpacePlantTest, UnstableSystem) {
  auto a = as_matrix<2, 2>({{1, .05}, {0, 1.03}});
  auto b = as_matrix<2, 1>({{0}, {.05}});
  auto c = as_matrix<1, 2>({{1, 0}});
  StateSpacePlant<1, 2, 1, true> flywheel_plant_(a, b, c);
  for (int i = 0; i < 100; i++) {
    auto u = as_matrix<1, 1>({{1}});
    flywheel_plant_.Update(u);
  }
  for (int i = 0; i < 1000; i++) {
    auto u = as_matrix<1, 1>({{0}});
    flywheel_plant_.Update(u);
  }
  ASSERT_GT(flywheel_plant_.GetX()(1), 1e10);
}

TEST(StateSpacePlantTest, SteadyState) {
  auto a = as_matrix<2, 2>({{1, .05}, {0, 0.97}});
  auto b = as_matrix<2, 1>({{0}, {.05}});
  auto c = as_matrix<1, 2>({{1, 0}});
  StateSpacePlant<1, 2, 1, true> flywheel_plant_(a, b, c);
  for (int i = 0; i < 1000; i++) {
    auto u = as_matrix<1, 1>({{1}});
    flywheel_plant_.Update(u);
  }
  ASSERT_NEAR(flywheel_plant_.GetX()(1), 5.0 / 3.0, 1e-10);
}

TEST(StateSpaceControllerTest, GoesToZero) {
  auto a = as_matrix<2, 2>({{1, .05}, {0, 0.9}});
  auto b = as_matrix<2, 1>({{0}, {.05}});
  auto c = as_matrix<1, 2>({{1, 0}});

  StateSpacePlant<1, 2, 1, true> plant(a, b, c);

  auto k = as_matrix<1, 2>({{1, 1}});
  StateFeedbackController<2, 1> controller(k);

  auto x0 = as_matrix<2, 1>({{1}, {3}});
  plant.SetX(x0);
  for (int i = 0; i < 1000; i++) {
    auto u = controller.Calculate(plant.GetX());
    plant.Update(u);
  }
  ASSERT_NEAR(plant.GetX()(0), 0.0, 1e-5);
  ASSERT_NEAR(plant.GetX()(1), 0.0, 1e-5);
}

TEST(StateSpaceControllerTest, GoesToGoal) {
  auto a = as_matrix<2, 2>({{1, .05}, {0, .9}});
  auto b = as_matrix<2, 1>({{0}, {.05}});
  auto c = as_matrix<1, 2>({{1, 0}});

  StateSpacePlant<1, 2, 1, true> plant(a, b, c);

  auto k = as_matrix<1, 2>({{1, 1}});
  StateFeedbackController<2, 1> controller(k);

  auto x0 = as_matrix<2, 1>({{1}, {3}});
  plant.SetX(x0);

  auto r = as_matrix<2, 1>({{1}, {0}});
  controller.SetGoal(r);

  for (int i = 0; i < 1000; i++) {
    auto u = controller.Calculate(plant.GetX());
    plant.Update(u);
  }
  ASSERT_NEAR(plant.GetX()(0), r(0), 1e-5);
  ASSERT_NEAR(plant.GetX()(1), r(1), 1e-5);
}

TEST(DiscretizeSystem, FollowsSimilarPath) {
  auto a = as_matrix<2, 2>({{0.0, 1.0}, {0.0, -1.0}});
  auto b = as_matrix<2, 1>({{0}, {1}});
  auto c = as_matrix<1, 2>({{3, 0}});
  Time dt = .001;
  auto sys = StateSpacePlant<1, 2, 1>(a, b, c);
  auto sys_d = c2d(sys, dt);
  for (Time t = 0; t <= 100; t += dt) {
    auto u = as_matrix<1, 1>({{1}});
    sys.Update(u, dt);
    sys_d.Update(u);
  }
  ASSERT_NEAR(sys.GetX()(0), sys.GetX()(0), 1e-5);
  ASSERT_NEAR(sys.GetX()(1), sys.GetX()(1), 1e-5);
}
