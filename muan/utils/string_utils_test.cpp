#include <cstring>
#include "gtest/gtest.h"
#include "muan/utils/string_utils.h"

TEST(CamelToSnake, NoChanges) {
  const char* str = "hello_world";
  char buffer[1024];
  size_t num_bytes_written = muan::utils::CamelToSnake(str, std::strlen(str) + 1, &buffer[0], 1024);
  EXPECT_STREQ(str, buffer);
  EXPECT_EQ(num_bytes_written, std::strlen(str));
}

TEST(CamelToSnake, Works) {
  const char* str = "HelloWorld";
  char buffer[1024];
  size_t num_bytes_written = muan::utils::CamelToSnake(str, std::strlen(str) + 1, &buffer[0], 1024);

  const char* expected_str = "hello_world";
  EXPECT_STREQ(expected_str, buffer);
  EXPECT_EQ(num_bytes_written, std::strlen(expected_str));
}
