#include "gtest/gtest.h"
#include "muan/utils/history.h"

using muan::utils::History;

TEST(History, Sizing) {
  History<int> hist(100);

  // Fill it with 99 samples
  for (int i = 0; i < 99; i++) {
    EXPECT_EQ(hist.num_samples(), i);
    EXPECT_FALSE(hist.is_full());
    hist.Update(0);
  }

  EXPECT_EQ(hist.num_samples(), 99);
  EXPECT_FALSE(hist.is_full());

  // Give it a 100th sample
  hist.Update(0);

  EXPECT_EQ(hist.num_samples(), 100);
  EXPECT_TRUE(hist.is_full());

  // Make sure it doesn't think we have >100 samples!
  hist.Update(0);

  EXPECT_EQ(hist.num_samples(), 100);
  EXPECT_TRUE(hist.is_full());
}

TEST(History, Iterates) {
  History<int> hist(100);
  for (int i = 0; i < 100; i++) {
    hist.Update(i);
  }

  int next = 0;
  for (int i : hist) {
    EXPECT_EQ(i, next);
    next++;
  }
  EXPECT_EQ(next, 100);
}

TEST(History, GoBack) {
  History<int> hist(100);

  for (int i = 0; i < 50; i++) {
    hist.Update(i);
  }

  for (int i = 0; i < 50; i++) {
    EXPECT_EQ(hist.GoBack(i), 49 - i);
  }
}

TEST(History, GoBackFull) {
  History<int> hist(100);

  for (int i = 0; i < 100; i++) {
    hist.Update(i);
  }

  for (int i = 0; i < 100; i++) {
    EXPECT_EQ(hist.GoBack(i), 99 - i);
  }
}

TEST(History, OverwriteOldHistory) {
  History<int> hist(100);

  for (int i = 0; i < 200; i++) {
    hist.Update(i);
  }

  for (int i = 0; i < 100; i++) {
    EXPECT_EQ(hist.GoBack(i), 199 - i);
  }

  int next = 100;
  for (int i : hist) {
    EXPECT_EQ(i, next);
    next = i + 1;
  }
  EXPECT_EQ(next, 200);
}

TEST(History, FailsOnOldHistory) {
  History<int> hist(100);

  EXPECT_DEATH(hist.GoBack(0), "unrecorded history");
  for (int i = 0; i < 100; i++) {
    hist.Update(i);
  }
  hist.GoBack(99);
  EXPECT_DEATH(hist.GoBack(100), "unrecorded history");
}
