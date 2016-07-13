#include "message_queue.h"
#include "gtest/gtest.h"
#include <thread>

using namespace muan::queues;

uint64_t factorial(uint64_t a) {
  if (a == 0) return 1;
  return a * factorial(a - 1);
}

TEST(MessageQueue, DeliversSingleMessage) {
  MessageQueue<uint32_t, 10> int_queue;
  int_queue.WriteMessage(10);
  auto reader = int_queue.MakeReader();
  ASSERT_EQ(reader.ReadMessage().value(), 10);
}

TEST(MessageQueue, DeliversManyMessages) {
  MessageQueue<uint32_t, 10> int_queue;
  auto reader = int_queue.MakeReader();
  for (uint32_t i = 1; i <= 10; i++) {
    int_queue.WriteMessage(i);
  }
  uint64_t accum = 1;
  std::experimental::optional<uint32_t> val;
  while (val = reader.ReadMessage()) {
    accum *= val.value();
  }
  ASSERT_EQ(accum, factorial(10));
}

TEST(MessageQueue, Wraparound) {
  MessageQueue<uint32_t, 10> int_queue;
  auto reader = int_queue.MakeReader();
  for (uint32_t i = 1; i <= 10; i++) {
    int_queue.WriteMessage(i);
  }
  uint64_t accum = 1;
  std::experimental::optional<uint32_t> val;
  while (val = reader.ReadMessage()) {
    accum *= val.value();
  }

  for (uint32_t i = 11; i <= 15; i++) {
    int_queue.WriteMessage(i);
  }
  while (val = reader.ReadMessage()) {
    accum *= val.value();
  }
  ASSERT_EQ(accum, factorial(15));
}

TEST(MessageQueue, TwoReaders) {
  MessageQueue<uint32_t, 10> int_queue;
  auto reader1 = int_queue.MakeReader();
  auto reader2 = int_queue.MakeReader();
  for (uint32_t i = 1; i <= 10; i++) {
    int_queue.WriteMessage(i);
  }

  uint64_t accum1 = 1;
  std::experimental::optional<uint32_t> val;
  while (val = reader1.ReadMessage()) {
    accum1 *= val.value();
  }

  uint64_t accum2 = 1;
  while (val = reader2.ReadMessage()) {
    accum2 *= val.value();
  }
  ASSERT_EQ(accum1, factorial(10));
  ASSERT_EQ(accum2, factorial(10));
}

TEST(MessageQueue, SpeedTest) {
  MessageQueue<uint32_t, 1000000> int_queue;
  auto reader1 = int_queue.MakeReader();
  auto reader2 = int_queue.MakeReader();
  for (uint32_t i = 1; i <= 1000000; i++) {
    int_queue.WriteMessage(i);
  }

  uint64_t accum1 = 1;
  std::experimental::optional<uint32_t> val;
  while (val = reader1.ReadMessage()) {
    accum1++;
  }

  uint64_t accum2 = 1;
  while (val = reader2.ReadMessage()) {
    accum2++;
  }
  ASSERT_EQ(accum1, 1000000);
  ASSERT_EQ(accum2, 1000000);
}
