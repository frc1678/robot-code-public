#include <sstream>
#include "muan/logging/textlogger.h"
#include "gtest/gtest.h"
#include "muan/queues/message_queue.h"

TEST(TextLogger, PassesMessage) {
  muan::utils::SetMocktimeEpoch();
  std::stringstream stream;
  muan::logging::TextLogger logger;
  logger.Log(__FILE__, __LINE__, "_%d_", 1);
  aos::time::IncrementMockTime(std::chrono::seconds(1));
  logger.Log(__FILE__, __LINE__, "_%s_", "foo");
  auto reader = logger.MakeReader();
  reader.ReadMessage()->message(stream);
  reader.ReadMessage()->message(stream);
  EXPECT_FALSE(reader.ReadMessage());
  ASSERT_EQ(stream.str(), "0:muan/logging/textlogger_test.cpp:10: _1_\n"
                          "1000:muan/logging/textlogger_test.cpp:12: _foo_\n");
}

TEST(TextLogger, LongMessage) {
  muan::utils::SetMocktimeEpoch();
  std::stringstream stream;
  muan::logging::TextLogger logger;
  char text[2001];
  memset(text, 'a', 2000);
  text[2000] = '\0';
  logger.Log(__FILE__, __LINE__, text);
  auto reader = logger.MakeReader();
  reader.ReadMessage()->message(stream);
  EXPECT_FALSE(reader.ReadMessage());
  //  trims to buffer length
  ASSERT_EQ(stream.str(), "0:muan/logging/textlogger_test.cpp:28: "
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
  logger.Log(__FILE__, __LINE__, "_%d_", 1);
  EXPECT_EQ(*logger.MakeReader().ReadMessage()->thread_name, "textlogging");
}

TEST(TextLogger, StackString) {
  muan::utils::SetMocktimeEpoch();
  std::stringstream stream;
  muan::logging::TextLogger logger;
  {
    std::string str = "Stack Attack";
    logger.Log(__FILE__, __LINE__, str.c_str());
    str = "Recycle Rush";
    logger.Log(__FILE__, __LINE__, "%s", str.c_str());
    str = "Power Up";
  }
  auto reader = logger.MakeReader();
  reader.ReadMessage()->message(stream);
  reader.ReadMessage()->message(stream);
  ASSERT_EQ(stream.str(), "0:muan/logging/textlogger_test.cpp:69: Stack Attack\n"
                          "0:muan/logging/textlogger_test.cpp:71: Recycle Rush\n");
}
