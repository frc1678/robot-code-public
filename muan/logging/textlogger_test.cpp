#include <sstream>
#include "muan/logging/textlogger.h"
#include "gtest/gtest.h"
#include "muan/queues/message_queue.h"

TEST(TextLogger, PassesMessageStream) {
  muan::utils::SetMocktimeEpoch();
  std::stringstream stream;
  muan::logging::TextLogger logger;
  logger.LogStream(__FILE__, __LINE__, "aaaa", 1);
  aos::time::IncrementMockTime(std::chrono::seconds(1));
  logger.LogStream(__FILE__, __LINE__, "aaaa", "xz");
  auto reader = logger.MakeReader();
  reader.ReadMessage()->message(stream);
  reader.ReadMessage()->message(stream);
  EXPECT_FALSE(reader.ReadMessage());
  ASSERT_EQ(stream.str(), "0:muan/logging/textlogger_test.cpp:10: aaaa1\n"
                          "1000:muan/logging/textlogger_test.cpp:12: aaaaxz\n");
}

TEST(TextLogger, LongMessageStream) {
  muan::utils::SetMocktimeEpoch();
  std::stringstream stream;
  muan::logging::TextLogger logger;
  char text[2001];
  memset(text, 'a', 2000);
  text[2000] = '\0';
  logger.LogStream(__FILE__, __LINE__, text);
  auto reader = logger.MakeReader();
  reader.ReadMessage()->message(stream);
  EXPECT_FALSE(reader.ReadMessage());
  ASSERT_EQ(stream.str(), "0:muan/logging/textlogger_test.cpp:28: "
// 100 'a's per line, 20 lines. Count them, if you hate yourself.
"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
"\n");
}

TEST(TextLogger, PassesMessagePrintf) {
  muan::utils::SetMocktimeEpoch();
  std::stringstream stream;
  muan::logging::TextLogger logger;
  logger.LogPrintf(__FILE__, __LINE__, "_%d_", 1);
  aos::time::IncrementMockTime(std::chrono::seconds(1));
  logger.LogPrintf(__FILE__, __LINE__, "_%s_", "foo");
  auto reader = logger.MakeReader();
  reader.ReadMessage()->message(stream);
  reader.ReadMessage()->message(stream);
  EXPECT_FALSE(reader.ReadMessage());
  ASSERT_EQ(stream.str(), "0:muan/logging/textlogger_test.cpp:61: _1_\n"
                          "1000:muan/logging/textlogger_test.cpp:63: _foo_\n");
}

TEST(TextLogger, LongMessagePrintf) {
  muan::utils::SetMocktimeEpoch();
  std::stringstream stream;
  muan::logging::TextLogger logger;
  char text[2001];
  memset(text, 'a', 2000);
  text[2000] = '\0';
  logger.LogPrintf(__FILE__, __LINE__, text);
  auto reader = logger.MakeReader();
  reader.ReadMessage()->message(stream);
  EXPECT_FALSE(reader.ReadMessage());
  // Printf trims to buffer length
  ASSERT_EQ(stream.str(), "0:muan/logging/textlogger_test.cpp:79: "
// 64 'a's per line, 16 lines. Count them, if you hate yourself.
"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"  // terminating null
"\n");
}

TEST(TextLogger, ThreadName) {
  muan::utils::SetMocktimeEpoch();
  muan::utils::SetCurrentThreadName("textlogging");
  std::stringstream stream;
  muan::logging::TextLogger logger;
  logger.LogPrintf(__FILE__, __LINE__, "_%d_", 1);
  EXPECT_EQ(*logger.MakeReader().ReadMessage()->thread_name, "textlogging");
}
