#include "gtest/gtest.h"
#include "muan/utils/history.h"

using muan::utils::History;

TEST(History, WorksCorrectly) {
  using namespace muan::units;
  History<int, 200> hist(.01 * s);
  for (int i = 0; i < 100; i++) {
    hist.Update(i);
  }

  // We don't really care about off-by-one errors
  for (Time t = .01 * s; t < 1 * s; t += .01 * s) {  // NOLINT
    EXPECT_NEAR(hist.GoBack(t), 100 - static_cast<int>(convert(t, .01 * s)), 1);
  }
}

TEST(History, FailsOnOldHistory) {
  using namespace muan::units;
  History<int, 200> hist(.01 * s);

  EXPECT_DEATH(hist.GoBack(250 * 0.01 * s), "unrecorded history");
}
