#include "trapezoidal_motion_profile.h"
#include "utils/math_utils.h"
#include "unitscpp/unitscpp.h"
#include "gtest/gtest.h"
#include <fstream>

using namespace muan::control;

TEST(TrapezoidalMotionProfileTest, IsSane) {
  muan::control::MotionProfileConstraints<Length> constraints{1 * m / s,
                                                              1 * m / s / s};
  muan::control::MotionProfilePosition<Length> goal{1 * m, 0 * m / s};
  muan::control::TrapezoidalMotionProfile<Length> profile{constraints, goal}; 

  muan::control::MotionProfilePosition<Length> result{0 * m, 0 * m / s};

  Time t = 0 *s;

  result = profile.Calculate(t);

  ASSERT_TRUE(result.position == 0 * m);
  ASSERT_TRUE(result.velocity == 0 * m / s);
}

TEST(TrapezoidalMotionProfileTest, ReachesGoal) {
  muan::control::MotionProfileConstraints<Length> constraints{1 * m / s,
                                                              1 * m / s / s};
  muan::control::MotionProfilePosition<Length> goal{1 * m, 0 * m / s};
  muan::control::MotionProfilePosition<Length> result{0 * m, 0 * m / s};
  muan::control::TrapezoidalMotionProfile<Length> profile{constraints, goal}; 

  const Time dt = 0.005 * s;
  Time t = 0 *s;

  while(!profile.finished(t)) {
    result = profile.Calculate(t);
    t += dt;
  }

  ASSERT_TRUE(result.position == 1 * m);
  ASSERT_TRUE(result.velocity == 0 * m / s);
}

TEST(TrapezoidalMotionProfileTest, StaysInBounds) {
  muan::control::MotionProfileConstraints<Length> constraints{1 * m / s,
                                                              1 * m / s / s};
  muan::control::MotionProfilePosition<Length> goal{3 * m, 0 * m / s};
  muan::control::MotionProfilePosition<Length> result{0 * m, 0 * m / s};
  muan::control::TrapezoidalMotionProfile<Length> profile{constraints, goal}; 

  muan::Differentiator<Velocity> dvdt;
  muan::Differentiator<Length> dxdt;

  const Time dt = 0.005 * s;
  const Velocity discreete_error = 0.0026 * m / s;

  Time t = 0 *s;

  bool velocity_stayed_in_bounds = true;
  bool estimated_velocity_follows_velocity = true;
  bool acceleration_stayed_in_bounds = true;

  while(!profile.finished(t)) {
    result = profile.Calculate(t);
    Acceleration estimated_acceleration = dvdt.differentiate(result.velocity, dt);
    Velocity estimated_velocity = dxdt.differentiate(result.position, dt);

    if(muan::abs(result.velocity) > 1 * m / s) {
      velocity_stayed_in_bounds = false;
    }
    if(muan::abs(estimated_acceleration) > 1 * m / s / s + (discreete_error / s)) {
      acceleration_stayed_in_bounds = false;
    }
    if(muan::abs(estimated_velocity) > result.velocity + discreete_error) {
      estimated_velocity_follows_velocity = false;
    } 

    t += dt;
  }

  ASSERT_TRUE(velocity_stayed_in_bounds);
  ASSERT_TRUE(estimated_velocity_follows_velocity);
  ASSERT_TRUE(acceleration_stayed_in_bounds);
}

TEST(TrapezoidalMotionProfileTest, NonZeroInitial) {
  muan::control::MotionProfileConstraints<Length> constraints{1 * m / s,
                                                              1 * m / s / s};
  muan::control::MotionProfilePosition<Length> goal{3 * m, 0 * m / s};
  muan::control::MotionProfilePosition<Length> result{-1 * m, -10 * m / s};
  muan::control::TrapezoidalMotionProfile<Length> profile{constraints, goal, result}; 

  muan::Differentiator<Velocity> dvdt;
  muan::Differentiator<Length> dxdt;

  const Time dt = 0.005 * s;
  const Velocity discreete_error = 0.0026 * m / s;

  Time t = 0 *s;

  bool velocity_stayed_in_bounds = true;
  bool estimated_velocity_follows_velocity = true;
  bool acceleration_stayed_in_bounds = true;

  while(!profile.finished(t)) {
    result = profile.Calculate(t);
    Acceleration estimated_acceleration = dvdt.differentiate(result.velocity, dt);
    Velocity estimated_velocity = dxdt.differentiate(result.position, dt);

    if(muan::abs(result.velocity) > 1 * m / s) {
      velocity_stayed_in_bounds = false;
    }
    if(muan::abs(estimated_acceleration) > 1 * m / s / s + (discreete_error / s)) {
      acceleration_stayed_in_bounds = false;
    }
    if(muan::abs(estimated_velocity) > result.velocity + discreete_error) {
      estimated_velocity_follows_velocity = false;
    } 

    std::cout << std::to_string(t.to(s)) << ", " << std::to_string(result.position.to(m)) << ", " << std::to_string(result.velocity.to(m/s)) << ", " << std::to_string(estimated_velocity.to(m/s)) << ", " << std::to_string(estimated_acceleration.to(m/s/s)) << std::endl;

    t += dt;
  }

  ASSERT_TRUE(result.position == 3 * m);
  ASSERT_TRUE(result.velocity == 0 * m / s);
  ASSERT_TRUE(velocity_stayed_in_bounds);
  ASSERT_TRUE(estimated_velocity_follows_velocity);
  ASSERT_TRUE(acceleration_stayed_in_bounds);

}
