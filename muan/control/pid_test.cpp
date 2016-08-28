#include "pid_controller.h"
#include "gtest/gtest.h"

using namespace muan;

TEST(PidController, GoesToPosition) {
  using namespace muan::units;
  PidController p(1 * V / m, 1 * V / m / s, 0 * V * s / m);
  Meters h = 1.0 * m;
  for (int i = 0; i < 10000; i++) {
    Seconds dt = s / 1000;
    Volts volt = p.Calculate(dt, -h);
    h += convert(volt, V) * dt * (m / s);
  }
  EXPECT_NEAR(convert(h, m), 0, .05);
}
