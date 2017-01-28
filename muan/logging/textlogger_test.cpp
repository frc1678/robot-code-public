#include "muan/logging/textlogger.h"
#include "gtest/gtest.h"
#include "muan/queues/message_queue.h"

TEST(TextLogger, PassesMessage) {
  aos::time::EnableMockTime(aos::monotonic_clock::epoch());
  muan::logging::TextLogger::TextQueuePtr queue = std::make_shared<muan::logging::TextLogger::TextQueue>();
  auto reader = queue->MakeReader();
  muan::logging::TextLogger logger(queue);
  logger("aaaa");
  ASSERT_EQ(std::string(&reader.ReadMessage().value()[0]), "0,aaaa");
  aos::time::IncrementMockTime(std::chrono::seconds(1));
  logger("aaaa");
  ASSERT_EQ(std::string(&reader.ReadMessage().value()[0]), "1000,aaaa");
}

TEST(TextLogger, MessageTooLong) {
  aos::time::EnableMockTime(aos::monotonic_clock::epoch());
  muan::logging::TextLogger::TextQueuePtr queue = std::make_shared<muan::logging::TextLogger::TextQueue>();
  auto reader = queue->MakeReader();
  muan::logging::TextLogger logger(queue);
  char text[2000];
  memset(text, 'a', 2000);
  text[1999] = '\0';
  logger(text);
  ASSERT_EQ(std::string(&reader.ReadMessage().value()[0]),
            "0,"
            "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
            "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
            "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
            "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
            "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
            "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
            "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
            "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
            "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
            "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
            "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
}
