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
  ASSERT_EQ(std::string(&reader.ReadMessage().value()[0]), "0,aaaa");
  aos::time::Time::SetMockTime(aos::time::Time::InSeconds(1));
  logger("aaaa");
  ASSERT_EQ(std::string(&reader.ReadMessage().value()[0]), "1000,aaaa");
}

TEST(TextLogger, MessageTooLong) {
  aos::time::Time::EnableMockTime();
  muan::logging::TextLogger::TextQueuePtr queue = std::make_shared<muan::logging::TextLogger::TextQueue>();
  auto reader = queue->MakeReader();
  muan::logging::TextLogger logger(queue);
  aos::time::Time::SetMockTime(aos::time::Time::InSeconds(0));
  char text[2000];
  memset(text, 'a', 2000);
  text[1999] = '\0';
  logger(text);
  ASSERT_EQ(std::string(&reader.ReadMessage().value()[0]), "0,aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
}
