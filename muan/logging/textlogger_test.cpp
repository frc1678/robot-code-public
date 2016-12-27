#include "gtest/gtest.h"
#include "textlogger.h"
#include "muan/queues/message_queue.h"
#include <iostream>

TEST(TextLogger, PassesMessage) {
  aos::time::Time::EnableMockTime();
  muan::logging::TextLogger::TextQueuePtr queue = std::make_shared<muan::logging::TextLogger::TextQueue>();
  auto reader = queue->MakeReader();
  muan::logging::TextLogger logger(queue);
  aos::time::Time::SetMockTime(aos::time::Time::InSeconds(0));
  logger("aaaa");
  ASSERT_EQ(reader.ReadMessage().value_or("none"), "0,aaaa");
  aos::time::Time::SetMockTime(aos::time::Time::InSeconds(1));
  logger("aaaa");
  ASSERT_EQ(reader.ReadMessage().value_or("none"), "1000,aaaa");
}
