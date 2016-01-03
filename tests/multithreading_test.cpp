#include "gtest/gtest.h"
#include "../multithreading/updateable.h"
#include "../utils/timing_utils.h"

class TestUpdateable : public Updateable, public testing::Test {
 public:
  TestUpdateable() : Updateable(500 * hz) {}
  int x() { return _x; }

 protected:
  void SetUp() {
    sleep_for(.001*s);
  }

  void TearDown() {}

  void Update(Time dt) override {
    if (_x > 0) _x--;
  }
  int _x = 300;
};

TEST_F(TestUpdateable, BasicTest) {
  Start();
  sleep_for(.7*s);
  Stop();
  ASSERT_EQ(_x, 0);
}

TEST_F(TestUpdateable, Stop) {
  Start();
  sleep_for(.1*s);
  Stop();
  ASSERT_GT(_x, 0);
}

TEST_F(TestUpdateable, StopsPromptly) {
  Start();
  // The countdown takes .6 seconds total at 500hz, so it should not finish if
  // left for .59 seconds - it should finish very quickly when called.
  sleep_for(.595*s);
  Stop();
  ASSERT_GT(_x, 0);
}
