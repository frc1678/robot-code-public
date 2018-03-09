#include "muan/queues/message_queue.h"
#include <thread>
#include "gtest/gtest.h"
#include "muan/proto/stack_proto.h"
#include "muan/queues/queue_manager.h"
#include "muan/queues/test_proto.pb.h"

using muan::queues::MessageQueue;
using muan::queues::QueueManager;

// Ensure that the queue delivers a single message correctly
TEST(MessageQueue, DeliversSingleMessage) {
  MessageQueue<uint32_t> int_queue(10);
  int_queue.WriteMessage(10);
  auto reader = int_queue.MakeReader();
  EXPECT_EQ(reader.ReadMessage().value(), 10);
}

// Ensure that the queue reads the last message correctly through it's public
// API
TEST(MessageQueue, QueueReadsLastMessage) {
  MessageQueue<uint32_t> int_queue(10);
  // We haven't written anything, so expect nullopt
  EXPECT_EQ(int_queue.ReadLastMessage(), std::experimental::nullopt);
  int_queue.WriteMessage(254);
  int_queue.WriteMessage(971);
  int_queue.WriteMessage(1678);
  // Expect reading the last value, but not "consuming" it (unlike a
  // QueueReader)
  EXPECT_EQ(int_queue.ReadLastMessage().value(), 1678);
  EXPECT_EQ(int_queue.ReadLastMessage().value(), 1678);
}

// Ensure that the queue reader reads the last message correctly
TEST(MessageQueue, ReaderReadsLastMessage) {
  MessageQueue<uint32_t> int_queue(10);
  int_queue.WriteMessage(254);
  int_queue.WriteMessage(971);
  int_queue.WriteMessage(1678);
  auto reader = int_queue.MakeReader();
  // Check that ReadLastMessage actually reads the last message
  EXPECT_EQ(reader.ReadLastMessage().value(), 1678);
  // Check that ReadLastMessage will move the current message pointer to the
  // front.
  EXPECT_FALSE(reader.ReadMessage());
}

// Ensure that the queue delivers multiple messages correctly and in sequence
TEST(MessageQueue, DeliversManyMessages) {
  MessageQueue<uint32_t> int_queue(10);
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
  MessageQueue<uint32_t> int_queue(10);
  auto reader = int_queue.MakeReader();

  // Make sure that it doesn't have any messages, because it's empty
  EXPECT_FALSE(reader.ReadLastMessage());

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
  MessageQueue<uint32_t> int_queue(10);

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
  MessageQueue<uint32_t> int_queue(1000000);
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
  MessageQueue<uint32_t> int_queue(num_messages);
  auto func = [&int_queue, num_messages]() {
    uint32_t next = 0;
    auto reader = int_queue.MakeReader();
    auto timeout_end =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(1000);

    while (next < num_messages &&
           std::chrono::steady_clock::now() < timeout_end) {
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

// Ensure that the queues maintain correctness when being accessed by many
// reader threads and multiple writer threads
TEST(MessageQueue, MultipleWriters) {
  constexpr uint32_t messages_per_thread = 2000;
  constexpr uint32_t num_threads = 5;

  MessageQueue<uint32_t> int_queue(messages_per_thread * num_threads);
  auto reader_func = [&int_queue]() {
    auto reader = int_queue.MakeReader();

    uint32_t num_read = 0;
    auto timeout_end =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(1000);

    while (num_read < messages_per_thread * num_threads &&
           std::chrono::steady_clock::now() < timeout_end) {
      if (reader.ReadMessage()) {
        num_read++;
      }
    }
    EXPECT_EQ(num_read, messages_per_thread * num_threads);
  };

  auto writer_func = [&int_queue]() {
    for (uint32_t i = 0; i < messages_per_thread; i++) {
      int_queue.WriteMessage(i);
    }
  };

  std::array<std::thread, num_threads> reader_threads;
  for (auto& t : reader_threads) {
    t = std::thread{reader_func};
  }

  std::array<std::thread, num_threads> writer_threads;
  for (auto& t : writer_threads) {
    t = std::thread{writer_func};
  }

  for (auto& t : reader_threads) {
    t.join();
  }

  for (auto& t : writer_threads) {
    t.join();
  }
}

TEST(MessageQueue, Reset) {
  MessageQueue<uint32_t> test_queue(10);
  auto reader = test_queue.MakeReader();
  test_queue.WriteMessage(0);
  EXPECT_TRUE(reader.ReadLastMessage());
  test_queue.Reset();
  EXPECT_FALSE(reader.ReadLastMessage());
}

TEST(MessageQueue, MessageIndex) {
  MessageQueue<uint32_t> test_queue(10);
  auto reader = test_queue.MakeReader();

  EXPECT_EQ(reader.GetNextMessageIndex(), 0);
  EXPECT_EQ(reader.GetNumMessagesSkipped(), 0);

  test_queue.WriteMessage(0);

  EXPECT_EQ(reader.GetNextMessageIndex(), 0);
  EXPECT_EQ(reader.GetNumMessagesSkipped(), 0);

  reader.ReadMessage();

  EXPECT_EQ(reader.GetNextMessageIndex(), 1);
  EXPECT_EQ(reader.GetNumMessagesSkipped(), 0);

  for (size_t i = 0; i < 11; i++) {
    test_queue.WriteMessage(0);
  }

  EXPECT_EQ(reader.GetNextMessageIndex(), 2);
  EXPECT_EQ(reader.GetNumMessagesSkipped(), 1);
}

TEST(MessageQueue, TimestampMessage) {
  muan::proto::StackProto<muan::queues::TimestampTestMessage, 256>
      stack_test_message;
  MessageQueue<muan::proto::StackProto<muan::queues::TimestampTestMessage, 256>>
      stack_test_queue(10);

  aos::time::EnableMockTime(aos::monotonic_clock::now());

  stack_test_queue.WriteMessage(stack_test_message);
  EXPECT_EQ(stack_test_queue.ReadLastMessage().value()->timestamp(),
            std::chrono::duration_cast<std::chrono::milliseconds>(
                aos::monotonic_clock::now() - aos::monotonic_clock::epoch())
                    .count() -
                muan::proto::start_time);
  muan::queues::TimestampTestMessage test_message;
  MessageQueue<muan::queues::TimestampTestMessage> test_queue(10);

  test_queue.WriteMessage(test_message);
  EXPECT_EQ(test_queue.ReadLastMessage().value().timestamp(),
            std::chrono::duration_cast<std::chrono::milliseconds>(
                aos::monotonic_clock::now() - aos::monotonic_clock::epoch())
                    .count() -
                muan::proto::start_time);
}

TEST(QueueManager, FetchQueue) {
  MessageQueue<int>* queue = QueueManager<int>::Fetch("test_queue");
  typename MessageQueue<int>::QueueReader reader =
      QueueManager<int>::Fetch("test_queue")->MakeReader();
  queue->WriteMessage(5);

  int msg;
  EXPECT_TRUE(reader.ReadLastMessage(&msg));
  EXPECT_EQ(msg, 5);
}

TEST(MessageQueue, GlobalReset) {
  MessageQueue<int>* queue = QueueManager<int>::Fetch("test_queue");
  queue->WriteMessage(5);

  ::muan::queues::ResetAllQueues();

  int msg;
  EXPECT_FALSE(QueueManager<int>::Fetch("test_queue")->ReadLastMessage(&msg));
}
