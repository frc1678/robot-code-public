#include "gtest/gtest.h"
#include "muan/control/pid_controller.h"

TEST(PidController, GoesToPosition) {
  using muan::units::V;
  using muan::units::m;
  using muan::units::s;
  using muan::units::Length;
  using muan::units::Time;
  using muan::units::Voltage;
  using muan::units::convert;
  muan::PidController p(1 * V / m, 1 * V / m / s, 0 * V * s / m);
  Length h = 1.0 * m;
  for (int i = 0; i < 10000; i++) {
    Time dt = s / 1000;
    Voltage volt = p.Calculate(dt, -h);
    h += convert(volt, V) * dt * (m / s);
  }
  EXPECT_NEAR(convert(h, m), 0 * m, .05 * m);
}
