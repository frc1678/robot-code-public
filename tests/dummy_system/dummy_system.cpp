#include "gtest/gtest.h"
#include <iostream>
#include <chrono>
#include <thread>
#include "test_subsystem.h"
#include "test_state_machine.h"

TEST(DummySystem, Works) {
  TestSubsystem sub;
  TestStateMachine tsm(sub);
  tsm.Start();
  sub.Start();
  for (int i = 0; i < 1200; i++) {
    tsm.Update();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  sub.Stop();
  ASSERT_EQ(sub.count_, 2);
}
