#include "unitscpp/unitscpp.h"
#include "gtest/gtest.h"
#include <iostream>

TEST(Units, ConvertUnits) {
  Angle a = 1 * rad;
  ASSERT_NEAR(a.to(deg), 57.2958, .001);
  ASSERT_NEAR(m.to(ft), 3.28084, .001);
  ASSERT_NEAR(s.to(hour), 1.0 / 3600.0, 0.001);
}
