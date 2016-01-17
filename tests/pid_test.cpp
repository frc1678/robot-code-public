#include "gtest/gtest.h"
#include "../control/pid_controller.h"

using namespace muan;

TEST(PidController, GoesToPosition) {
  PidController<Length, Voltage> p(1*V/m, 1*V/m/s, 0*V*s/m);
  Length h = 1.0*m;
  Velocity v = 0*m/s;
  for (int i = 0; i < 10000; i++) {
    Time dt = s/1000;
    Voltage volt = p.Calculate(dt, -h);
    h += volt.to(V)*dt*(m/s);
  }
  ASSERT_NEAR(h.to(m), 0, .05);
}
