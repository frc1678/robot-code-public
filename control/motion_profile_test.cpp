#include "trapezoidal_motion_profile.h"
#include "unitscpp/unitscpp.h"
#include "utils/math_utils.h"
#include "gtest/gtest.h"

using namespace muan::control;

class MotionProfileTest : public ::testing::Test {
 public:
  void SetUp() override {
    constraints.max_velocity = 1 * m / s;
    constraints.max_acceleration = 1 * m / s / s;
  }

  void RunTest() {
    muan::control::TrapezoidalMotionProfile<Length> profile{constraints, goal,
                                                            initial_position};

    const Time dt = 0.005 * s;
    const Velocity discreete_error =
        0.0026 * m /
        s;  // Discreete time differentiation leaves a bit of over/undershoot.

    EXPECT_NEAR(profile.Calculate(0 * s).position(),
                initial_position.position(), 1e-6);
    EXPECT_NEAR(profile.Calculate(0 * s).velocity(),
                initial_position.velocity(), 1e-6);

    for (Time t = 0 * s; t <= profile.total_time(); t += dt) {
      Acceleration estimated_acceleration =
          (profile.Calculate(t).velocity - profile.Calculate(t - dt).velocity) /
          (dt);
      Velocity estimated_velocity =
          (profile.Calculate(t).position - profile.Calculate(t - dt).position) /
          (dt);

      EXPECT_GE(constraints.max_velocity,
                muan::abs(profile.Calculate(t).velocity));
      EXPECT_GE(constraints.max_acceleration + (discreete_error / s),
                muan::abs(estimated_acceleration));
      EXPECT_NEAR(profile.Calculate(t).velocity(), estimated_velocity(),
                  discreete_error());

      t += dt;
    }

    EXPECT_NEAR(profile.Calculate(profile.total_time()).position(),
                goal.position(), 1e-5);
    EXPECT_NEAR(profile.Calculate(profile.total_time()).velocity(),
                goal.velocity(), 1e-5);
  }

  muan::control::MotionProfilePosition<Length> initial_position, result, goal;
  muan::control::MotionProfileConstraints<Length> constraints;
};

// Probbably too many tests :P
TEST_F(MotionProfileTest, PositiveGoal) {
  initial_position = {0 * m, 0 * m / s};
  goal = {3 * m, 0 * m / s};
  RunTest();
}

TEST_F(MotionProfileTest, NegativeGoal) {
  initial_position = {0 * m, 0 * m / s};
  goal = {-3 * m, 0 * m / s};
  RunTest();
}

TEST_F(MotionProfileTest, OnGoal) {
  initial_position = {2 * m, 0 * m / s};
  goal = {2 * m, 0 * m / s};
  RunTest();
}

TEST_F(MotionProfileTest, InitialVelocity) {
  initial_position = {0 * m, 0.5 * m / s};
  goal = {3 * m, 0 * m / s};
  RunTest();
}

TEST_F(MotionProfileTest, NegativeInitialVelocity) {
  initial_position = {0 * m, -0.5 * m / s};
  goal = {3 * m, 0 * m / s};
  RunTest();
}

TEST_F(MotionProfileTest, PositiveGoalVelocity) {
  initial_position = {0 * m, 0 * m / s};
  goal = {3 * m, 0.8 * m / s};
  RunTest();
}

TEST_F(MotionProfileTest, NegativeGoalVelocity) {
  initial_position = {0 * m, 0 * m / s};
  goal = {3 * m, -0.7 * m / s};
  RunTest();
}

TEST_F(MotionProfileTest, OnGoalPositiveVelocity) {
  initial_position = {2 * m, 1 * m / s};
  goal = {2 * m, 0 * m / s};
  RunTest();
}

TEST_F(MotionProfileTest, OnGoalNegativeVelocity) {
  initial_position = {2 * m, -1 * m / s};
  goal = {2 * m, 0 * m / s};
  RunTest();
}

TEST_F(MotionProfileTest, ZeroGoal) {
  initial_position = {2 * m, 0 * m / s};
  goal = {0 * m, 0 * m / s};
  RunTest();
}

TEST_F(MotionProfileTest, ZeroGoalPositiveGoalVelocity) {
  initial_position = {1 * m, 0 * m / s};
  goal = {0 * m, 0.6 * m / s};
  RunTest();
}

TEST_F(MotionProfileTest, ZeroGoalNegativeGoalVelocity) {
  initial_position = {2 * m, 0 * m / s};
  goal = {0 * m, -0.4 * m / s};
  RunTest();
}

TEST_F(MotionProfileTest, ShortPositiveDistance) {
  initial_position = {0 * m, 0 * m / s};
  goal = {0.5 * m, 0 * m / s};
  RunTest();
}

TEST_F(MotionProfileTest, ShortNegativeDistance) {
  initial_position = {0 * m, 0 * m / s};
  goal = {-0.5 * m, 0 * m / s};
  RunTest();
}

TEST_F(MotionProfileTest, ShortDistanceInitialVelocity) {
  initial_position = {0 * m, 1 * m / s};
  goal = {0.5 * m, 0 * m / s};
  RunTest();
}

TEST_F(MotionProfileTest, ShortDistanceNegativeInitalVelocity) {
  initial_position = {0 * m, -1 * m / s};
  goal = {0.5 * m, 0 * m / s};
  RunTest();
}

TEST_F(MotionProfileTest, ShortNegativeDistanceInitialVelocity) {
  initial_position = {0 * m, 0.8 * m / s};
  goal = {-0.5 * m, 0 * m / s};
  RunTest();
}

TEST_F(MotionProfileTest, ShortNegativeDistanceNegativeVelocity) {
  initial_position = {0 * m, -0.7 * m / s};
  goal = {-0.5 * m, 0 * m / s};
  RunTest();
}
