#include "gtest/gtest.h"
#include "muan/control/trapezoidal_motion_profile.h"
#include "muan/units/units.h"
#include "muan/utils/math_utils.h"

using muan::control::MotionProfile;
using muan::control::MotionProfileConstraints;
using muan::control::MotionProfilePosition;
using muan::control::TrapezoidalMotionProfile;

using namespace muan::units;  // NOLINT

class MotionProfileTest : public ::testing::Test {
 public:
  void SetUp() override {
    constraints.max_velocity = 1 * m / s;
    constraints.max_acceleration = 1 * m / s / s;
  }

  void RunTest() {
    muan::control::TrapezoidalMotionProfile profile{constraints, goal,
                                                    initial_position};
    const Time dt = 0.005 * s;
    const Velocity discrete_error = 0.0026 * m / s;

    bool timing_this_test = !(initial_position.velocity != 0. &&
                              initial_position.position == goal.position) &&
                            (std::abs(goal.velocity) < 1e-10);

    // Discrete time differentiation leaves a bit of over/undershoot.

    EXPECT_NEAR(profile.Calculate(0 * s).position, initial_position.position,
                1e-6 * m);
    EXPECT_NEAR(profile.Calculate(0 * s).velocity, initial_position.velocity,
                1e-6 * m / s);
    for (Time t = 0 * s; t < profile.total_time(); t += dt) {  // NOLINT
      Acceleration estimated_acceleration =
          (profile.Calculate(t).velocity - profile.Calculate(t - dt).velocity) /
          (dt);
      Velocity estimated_velocity =
          (profile.Calculate(t).position - profile.Calculate(t - dt).position) /
          (dt);

      if ((std::abs(profile.Calculate(t).position - goal.position) < 1e-3) &&
          timing_this_test) {
        EXPECT_NEAR(profile.TimeLeftUntil(profile.Calculate(t).position), t,
                    5e-2);
      }

      EXPECT_GE(constraints.max_velocity,
                std::abs(profile.Calculate(t).velocity));
      EXPECT_GE(constraints.max_acceleration + (discrete_error / s),
                std::abs(estimated_acceleration));
      EXPECT_NEAR(profile.Calculate(t).velocity, estimated_velocity,
                  discrete_error);

      t += dt;
    }

    EXPECT_NEAR(profile.Calculate(profile.total_time()).position, goal.position,
                1e-5 * m);
    EXPECT_NEAR(profile.Calculate(profile.total_time()).velocity, goal.velocity,
                1e-5 * m / s);
  }

  muan::control::MotionProfilePosition initial_position, result, goal;
  muan::control::MotionProfileConstraints constraints;
};

/*
 * Test many scenarios with combinations of:
 *  positive, negative or 0/small delta position
 *  positive, negative, or 0/small initial velocity
 *  positive, negative, or 0/small goal velocity
 */

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
