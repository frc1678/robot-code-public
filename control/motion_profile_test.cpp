#include "trapezoidal_motion_profile.h"
#include "unitscpp/unitscpp.h"
#include "gtest/gtest.h"
#include <fstream>

using namespace muan::control;

TEST(TrapezoidalMotionProfileTest, Builds) {
  muan::control::MotionProfileConstraints<Length> constraints{1 * m / s,
                                                              1 * m / s / s};
  muan::control::MotionProfilePosition<Length> goal{1 * m, 0 * m / s};
  muan::control::TrapezoidalMotionProfile<Length> profile{constraints, goal};
}
