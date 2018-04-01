#include "muan/utils/disk_brake.h"
#include "gtest/gtest.h"

using muan::DiskBrake;
using muan::units::s;

TEST(DiskBrake, Works) {
  aos::time::Time::EnableMockTime();
  // Start unlocked, 2s to lock
  DiskBrake brake(false, 2 * s);
  EXPECT_EQ(brake.Update(true), DiskBrake::UNLOCKED);
  // Locking it for more that 2s should lock it
  aos::time::Time::IncrementMockTime(aos::time::Time{3, 0});
  EXPECT_EQ(brake.Update(true), DiskBrake::LOCKED);
  // Changing it for less that 2s should leave it in between
  aos::time::Time::IncrementMockTime(aos::time::Time{1, 0});
  EXPECT_EQ(brake.Update(false), DiskBrake::CHANGING);
  // 2 more seconds should complete the change
  aos::time::Time::IncrementMockTime(aos::time::Time{2, 0});
  EXPECT_EQ(brake.Update(false), DiskBrake::UNLOCKED);
  // More time shouldn't change it
  aos::time::Time::IncrementMockTime(aos::time::Time{2, 0});
  EXPECT_EQ(brake.Update(false), DiskBrake::UNLOCKED);
}
