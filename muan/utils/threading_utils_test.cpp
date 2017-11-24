#include <thread>
#include <chrono>
#include "gtest/gtest.h"
#include "muan/utils/threading_utils.h"

int TestF(int x) { return x * 3; }

class TestA {
 public:
  explicit TestA(int x) { x_ = x; }
  int x_;
};

TEST(DeferCall, FuncPtr) {
  muan::utils::DeferCall<int, 128> f(TestF, 14);
  EXPECT_EQ(f(), 42);
}

TEST(DeferCall, Closure) {
  auto closure = []() { int x = 2; return [x]() { return TestA(x); }; }();
  muan::utils::DeferCall<TestA, 128> f(closure);
  EXPECT_EQ(f().x_, 2);
}

TEST(DeferCall, StdFunction) {
  muan::utils::DeferCall<int, 128> f(std::function<int(int)>(TestF), -1);
  EXPECT_EQ(f(), -3);
}

TEST(DeferCall, Defers) {
  double x = 0.1;
  muan::utils::DeferCall<void, 128> f([&](){ x *= 5; });
  EXPECT_EQ(x, 0.1);
  f();
  EXPECT_EQ(x, 0.5);
}

TEST(DeferCall, Copyable) {
  muan::utils::DeferCall<int, 128> f1;
  muan::utils::DeferCall<int, 128> f2([](){ return 42; });
  f1 = f2;
  EXPECT_EQ(f1(), 42);
  EXPECT_EQ(f2(), 42);
}

TEST(DeferCall, Curry) {
  muan::utils::DeferCall<int, 128, int> f([](int x, int y){ return x*y; }, 2);
  EXPECT_EQ(f(3), 6);
}

TEST(Timestamp, Standard) {
  int64_t t0 = muan::utils::Timestamp();
  EXPECT_LT(t0, 500);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  int64_t t1 = muan::utils::Timestamp();
  EXPECT_NEAR(t1 - t0, 100, 1);
}

TEST(Timestamp, MockTime) {
  muan::utils::SetMocktimeEpoch();
  aos::time::IncrementMockTime(std::chrono::seconds(1));
  EXPECT_EQ(muan::utils::Timestamp(), 1000);
}

TEST(ThreadName, All) {
  EXPECT_EQ(muan::utils::GetCurrentThreadName(), "unnamed_thread");
  muan::utils::SetCurrentThreadName("foo");
  EXPECT_EQ(muan::utils::GetCurrentThreadName(), "foo");
  std::thread([](){
    EXPECT_EQ(muan::utils::GetCurrentThreadName(), "unnamed_thread");
    muan::utils::SetCurrentThreadName("bar");
    EXPECT_EQ(muan::utils::GetCurrentThreadName(), "bar");
  }).join();
  EXPECT_EQ(muan::utils::GetCurrentThreadName(), "foo");
  muan::utils::SetCurrentThreadName("biz");
  EXPECT_EQ(muan::utils::GetCurrentThreadName(), "biz");
  EXPECT_DEATH(muan::utils::SetCurrentThreadName("this_name_is_too_long"),
               "thread name 'this_name_is_too_long' too long\n");
  EXPECT_EQ(muan::utils::GetCurrentThreadName(), "biz");
}
