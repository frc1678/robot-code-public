#include <utility>
#include <memory>
#include <string>
#include <map>
#include "muan/logging/logger.hpp"
#include "muan/logging/filewriter.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "muan/logging/logger_test.pb.h"
#include "muan/units/units.h"
#include "muan/proto/stack_proto.h"

using muan::logging::Logger;
using muan::queues::MessageQueue;
using namespace ::testing;  // NOLINT

namespace muan {
namespace logging {

class MockFileWriter : public muan::logging::FileWriter {
 public:
  MockFileWriter() : muan::logging::FileWriter("/") {}
  MOCK_METHOD2(WriteLine, void(const std::string &filename, const std::string &line));
  std::ostream& GetTextFile(const std::string& filename) { return mock_files[filename]; }
  std::map<std::string, std::stringstream> mock_files;
};

TEST(Logger, LogsOneMessage) {
  std::unique_ptr<muan::logging::MockFileWriter> writer = std::make_unique<muan::logging::MockFileWriter>();

  EXPECT_CALL(*writer, WriteLine("testqueue.csv", MatchesRegex("thing,queue_index\n42,.*"))).Times(1);

  Logger logger(std::move(writer));

  MessageQueue<muan::proto::StackProto<logging_test::Test1, 256>> mq(100);
  muan::proto::StackProto<logging_test::Test1, 256> msg;
  msg->set_thing(42);
  mq.WriteMessage(msg);
  logger.AddQueue("testqueue", &mq);

  logger.Update();
}

TEST(Logger, LogsManyMessages) {
  std::unique_ptr<muan::logging::MockFileWriter> writer = std::make_unique<muan::logging::MockFileWriter>();

  EXPECT_CALL(*writer, WriteLine("testqueue.csv", MatchesRegex("thing,queue_index\n42,.*"))).Times(1);
  EXPECT_CALL(*writer, WriteLine("testqueue.csv", MatchesRegex("42,.*"))).Times(41);

  Logger logger(std::move(writer));

  MessageQueue<muan::proto::StackProto<logging_test::Test1, 256>> mq(100);
  for (int i = 1; i <= 42; i++) {
    muan::proto::StackProto<logging_test::Test1, 256> msg;
    msg->set_thing(42);
    mq.WriteMessage(msg);
  }
  logger.AddQueue("testqueue", &mq);

  logger.Update();
}

TEST(Logger, LogsMultipleQueues) {
  std::unique_ptr<muan::logging::MockFileWriter> writer = std::make_unique<muan::logging::MockFileWriter>();

  EXPECT_CALL(*writer, WriteLine("testqueue1.csv", MatchesRegex("thing,queue_index\n42,.*"))).Times(1);
  EXPECT_CALL(*writer, WriteLine("testqueue2.csv", MatchesRegex("thing,queue_index\n42,.*"))).Times(1);

  Logger logger(std::move(writer));

  MessageQueue<muan::proto::StackProto<logging_test::Test1, 256>> mq1(100);
  muan::proto::StackProto<logging_test::Test1, 256> msg1;
  msg1->set_thing(42);
  mq1.WriteMessage(msg1);
  logger.AddQueue("testqueue1", &mq1);

  MessageQueue<muan::proto::StackProto<logging_test::Test1, 256>> mq2(100);
  muan::proto::StackProto<logging_test::Test1, 256> msg2;
  msg2->set_thing(42);
  mq2.WriteMessage(msg2);
  logger.AddQueue("testqueue2", &mq2);

  logger.Update();
}

TEST(Logger, LogsManyMessagesPerTick) {
  std::unique_ptr<muan::logging::MockFileWriter> writer = std::make_unique<muan::logging::MockFileWriter>();

  EXPECT_CALL(*writer, WriteLine("testqueue.csv", MatchesRegex("thing,queue_index\n42,.*"))).Times(1);
  EXPECT_CALL(*writer, WriteLine("testqueue.csv", MatchesRegex("42,.*"))).Times(9999);

  Logger logger(std::move(writer));

  MessageQueue<muan::proto::StackProto<logging_test::Test1, 256>> mq(20000);
  logger.AddQueue("testqueue", &mq);

  for (int n = 1; n <= 10; n++) {
    for (int i = 1; i <= 1000; i++) {
      muan::proto::StackProto<logging_test::Test1, 256> msg;
      msg->set_thing(42);
      mq.WriteMessage(msg);
    }
    logger.Update();
  }
}

TEST(Logger, DiesOnDuplicateQueues) {
  std::unique_ptr<muan::logging::MockFileWriter> writer = std::make_unique<muan::logging::MockFileWriter>();
  Logger logger(std::move(writer));

  MessageQueue<muan::proto::StackProto<logging_test::Test1, 256>> mq1(10);
  logger.AddQueue("testqueue", &mq1);

  MessageQueue<muan::proto::StackProto<logging_test::Test1, 256>> mq2(25);
  ASSERT_DEATH(logger.AddQueue("testqueue", &mq2), "with same name \"testqueue\"");
}

TEST(Logger, TextLogger) {
  std::unique_ptr<muan::logging::MockFileWriter> writer = std::make_unique<muan::logging::MockFileWriter>();
  Logger logger(std::move(writer));

  muan::utils::SetMocktimeEpoch();
  muan::utils::SetCurrentThreadName("name");
  aos::time::IncrementMockTime(std::chrono::seconds(1));
  LOG_P("test");

  logger.Update();
  EXPECT_EQ(static_cast<MockFileWriter*>(logger.writer_.get())->mock_files["name.log"].str(),
            "1000:muan/logging/logger_test.cpp:125: test\n");
}

}  // namespace logging
}  // namespace muan
