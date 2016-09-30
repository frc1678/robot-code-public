#include "message_queue.h"
#include "gtest/gtest.h"
#include <iostream>
#include <thread>

using muan::queues::MessageQueue;

// Ensure that the queue delivers a single message correctly
TEST(MessageQueue, DeliversSingleMessage) {
  MessageQueue<uint32_t, 10> int_queue;
  int_queue.WriteMessage(10);
  auto reader = int_queue.MakeReader();
  EXPECT_EQ(reader.ReadMessage().value(), 10);
}

// Ensure that the queue delivers multiple messages correctly and in sequence
TEST(MessageQueue, DeliversManyMessages) {
  MessageQueue<uint32_t, 10> int_queue;
  auto reader = int_queue.MakeReader();
  for (uint32_t i = 0; i < 10; i++) {
    int_queue.WriteMessage(i);
  }

  uint32_t next = 0;
  std::experimental::optional<uint32_t> val;
  while ((val = reader.ReadMessage())) {
    EXPECT_EQ(*val, next);
    next++;
  }
  EXPECT_EQ(next, 10);
}

// Ensure that correctness is maintained when the queue is filled and the
// message positions wrap around to the beginning of the buffer
TEST(MessageQueue, Wraparound) {
  MessageQueue<uint32_t, 10> int_queue;
  auto reader = int_queue.MakeReader();
  for (uint32_t i = 0; i < 10; i++) {
    int_queue.WriteMessage(i);
  }

  uint64_t next = 0;
  std::experimental::optional<uint32_t> val;
  while ((val = reader.ReadMessage())) {
    EXPECT_EQ(*val, next);
    next++;
  }

  for (uint32_t i = 10; i < 15; i++) {
    int_queue.WriteMessage(i);
  }

  while ((val = reader.ReadMessage())) {
    EXPECT_EQ(*val, next);
    next++;
  }

  EXPECT_EQ(next, 15);
}

// Ensure that the queue still works with multiple readers
TEST(MessageQueue, TwoReaders) {
  MessageQueue<uint32_t, 10> int_queue;

  auto reader1 = int_queue.MakeReader();
  auto reader2 = int_queue.MakeReader();

  for (uint32_t i = 0; i < 10; i++) {
    int_queue.WriteMessage(i);
  }

  uint64_t next1 = 0;
  std::experimental::optional<uint32_t> val;
  while ((val = reader1.ReadMessage())) {
    EXPECT_EQ(*val, next1);
    next1++;
  }

  uint64_t next2 = 0;
  while ((val = reader2.ReadMessage())) {
    EXPECT_EQ(*val, next2);
    next2++;
  }
  EXPECT_EQ(next1, 10);
  EXPECT_EQ(next2, 10);
}

// Test the queue's speed with large values. This test is mainly for curiosity's
// sake and will not fail if the queue fails to acheive some arbitrarily set
// standard
TEST(MessageQueue, SpeedTest) {
  MessageQueue<uint32_t, 1000000> int_queue;
  auto reader = int_queue.MakeReader();

  for (uint32_t i = 0; i < 1000000; i++) {
    int_queue.WriteMessage(i);
  }

  uint64_t next = 0;
  std::experimental::optional<uint32_t> val;
  while ((val = reader.ReadMessage())) {
    EXPECT_EQ(*val, next);
    next++;
  }

  EXPECT_EQ(next, 1000000);
}

// Ensure that the queues maintain correctness when being accessed by many
// reader threads and a single writer thread
TEST(MessageQueue, Multithreading) {
  constexpr uint32_t num_messages = 10000;
  MessageQueue<uint32_t, num_messages> int_queue;
  auto func = [&int_queue, num_messages]() {
    uint32_t next = 0;
    auto reader = int_queue.MakeReader();
    auto end_time =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(300);

    // TODO(Kyle) Find a better termination condition for this
    while (std::chrono::steady_clock::now() < end_time) {
      auto val = reader.ReadMessage();
      if (val) {
        EXPECT_EQ(next, *val);
        next++;
      }
    }
    EXPECT_EQ(next, num_messages);
  };

  std::array<std::thread, 5> threads;
  for (auto& t : threads) {
    t = std::thread{func};
  }

  for (uint32_t i = 0; i < num_messages; i++) {
    int_queue.WriteMessage(i);
  }

  for (auto& t : threads) {
    t.join();
  }
}

// Ensure that the queue works correctly, even when only reading the most recent
// message at any given point in time
TEST(MessageQueue, LatestMessage) {
  constexpr uint32_t num_messages = 10000;
  MessageQueue<int, num_messages> int_queue;
  auto func = [&int_queue, num_messages]() {
    int last = -1;
    auto reader = int_queue.MakeReader();
    auto end_time =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(300);

    // TODO(Kyle) Find a better termination condition for this
    while (std::chrono::steady_clock::now() < end_time) {
      auto val = reader.LatestMessage();
      if (val) {
        EXPECT_GE(*val, last);
        last = *val;
      }
    }
    EXPECT_EQ(last, num_messages - 1);
  };

  std::array<std::thread, 1> threads;
  for (auto& t : threads) {
    t = std::thread{func};
  }

  for (uint32_t i = 0; i < num_messages; i++) {
    int_queue.WriteMessage(i);
  }

  for (auto& t : threads) {
    t.join();
  }
}
