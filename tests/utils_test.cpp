#include "gtest/gtest.h"
#include "../utils/timer.h"
#include "../utils/timing_utils.h"

TEST(TimeUtils, TimerPositive) {
  Timer t;
  t.Start();
  for (int i = 0; i < 10000; i++) {
    t.Get();
  }
  ASSERT_GT(t.Get(), 0 * s);
}

TEST(TimeUtils, TimerReset) {
  Timer t;
  t.Start();
  for (int i = 0; i < 10000; i++) {
    t.Get();
  }
  t.Reset();
  ASSERT_LT(t.Get(), 0.1 * s);
}

TEST(TimeUtils, TimerAndDelay) {
  Timer t;
  t.Start();
  sleep_for(.2 * s);
  ASSERT_NEAR(t.Get().to(s), .2, .01);
}

TEST(TimeUtils, SleepUntil) {
  Time start = now();
  sleep_until(start + .5 * s);
  ASSERT_NEAR(now().to(s), start.to(s) + .5, .01);
}
