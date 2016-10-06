#include "gtest/gtest.h"
#include "textlogger.h"
#include "muan/queues/message_queue.h"
#include <iostream>

TEST(TextLogger, PassesMessage) {
  muan::logging::TextLogger::TextQueuePtr queue = std::make_shared<muan::logging::TextLogger::TextQueue>();
  auto reader = queue->MakeReader();
  muan::logging::TextLogger logger(queue);
  logger("aaaa");
  ASSERT_EQ(reader.ReadMessage().value_or("none"), "aaaa");
}
