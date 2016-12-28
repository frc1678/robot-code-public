#include "gtest/gtest.h"
#include "muan/utils/timer.h"
#include "muan/utils/timing_utils.h"

using muan::utils::Timer;
using muan::utils::sleep_for;
using muan::utils::sleep_until;
using muan::utils::now;


TEST(TimeUtils, TimerPositive) {
  using namespace muan::units;
  Timer t;
  t.Start();
  for (int i = 0; i < 10000; i++) {
    t.Get();
  }
  EXPECT_GT(t.Get(), 0 * s);
}

TEST(TimeUtils, TimerReset) {
  using namespace muan::units;
  Timer t;
  t.Start();
  for (int i = 0; i < 10000; i++) {
    t.Get();
  }
  t.Reset();
  EXPECT_LT(t.Get(), 0.1 * s);
}

TEST(TimeUtils, TimerAndDelay) {
  using namespace muan::units;
  Timer t;
  t.Start();
  sleep_for(.2 * s);
  EXPECT_NEAR(convert(t.Get(), s), .2, .01);
}

TEST(TimeUtils, SleepUntil) {
  using namespace muan::units;
  Time start = now();
  sleep_until(start + .5 * s);
  EXPECT_NEAR(convert(now(), s), convert(start, s) + .5, .01);
}
