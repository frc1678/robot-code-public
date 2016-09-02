#include "muan/utils/timing_utils.h"
#include "updateable.h"
#include "gtest/gtest.h"

using muan::Updateable;
using muan::sleep_for;

class TestUpdateable : public Updateable, public testing::Test {
 public:
  TestUpdateable() : Updateable(100 * hz) {}
  int x() { return _x; }

 protected:
  void SetUp() override {
    // Sleep for a bit to wait for the previous case to finish
    sleep_for(.1 * s);
  }

  void TearDown() override {}

  void Update(Time /*dt*/) override {
    if (_x > 0) {
      _x--;
    }
  }
  int _x = 100;
};

TEST_F(TestUpdateable, BasicTest) {
  Start();
  sleep_for(1.1 * s);
  Stop();
  EXPECT_EQ(_x, 0);
}

TEST_F(TestUpdateable, Stop) {
  Start();
  sleep_for(.1 * s);
  Stop();
  EXPECT_GT(_x, 0);
}

TEST_F(TestUpdateable, StopsPromptly) {
  Start();
  // The countdown takes .6 seconds total at 500hz, so it should not finish if
  // left for .55 seconds - it should finish very quickly when called.
  sleep_for(.55 * s);
  Stop();
  EXPECT_GT(_x, 0);
}
