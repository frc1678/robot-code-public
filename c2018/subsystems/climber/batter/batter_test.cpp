#include "c2018/subsystems/climber/batter/batter.h"
#include "gtest/gtest.h"

TEST(Batter, CanPutDownBatter) {
  c2018::climber::batter::Batter batter;
  EXPECT_TRUE(batter.Update(true, true));
}

TEST(Batter, NotPutDown) {
  c2018::climber::batter::Batter batter;
  EXPECT_FALSE(batter.Update(false, true));
}

TEST(Batter, Disabled) {
  c2018::climber::batter::Batter batter;
  EXPECT_FALSE(batter.Update(true, false));
}

TEST(Batter, DisabledAndNotPutDown) {
  c2018::climber::batter::Batter batter;
  EXPECT_FALSE(batter.Update(false, false));
}
