#include "pid_controller.h"
#include "gtest/gtest.h"

using muan::PidController;

TEST(PidController, GoesToPosition) {
  using namespace muan::units;
  PidController p(1 * V / m, 1 * V / m / s, 0 * V * s / m);
  Length h = 1.0 * m;
  for (int i = 0; i < 10000; i++) {
    Time dt = s / 1000;
    Voltage volt = p.Calculate(dt, -h);
    h += convert(volt, V) * dt * (m / s);
  }
  EXPECT_NEAR(convert(h, m), 0, .05);
}
