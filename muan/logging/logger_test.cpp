#include "logger.h"
#include "filewriter.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "muan/logging/logger_test.pb.h"
#include "muan/units/units.h"
#include "muan/proto/stack_proto.h"

using muan::logging::Logger;
using muan::queues::MessageQueue;
using namespace ::testing;

namespace muan {
namespace logging {

class MockFileWriter : public muan::logging::FileWriter {
 public:
  MockFileWriter() : muan::logging::FileWriter("/"){};
  MOCK_METHOD2(WriteLine, void(const std::string &filename, const std::string &line));
};

TEST(Logger, LogsOneMessage) {
  std::unique_ptr<muan::logging::MockFileWriter> writer = std::make_unique<muan::logging::MockFileWriter>();

  EXPECT_CALL(*writer, WriteLine("testqueue.csv", "42")).Times(1);

  Logger logger(std::move(writer));

  MessageQueue<muan::proto::StackProto<logging_test::Test1, 256>, 100> mq;
  muan::proto::StackProto<logging_test::Test1, 256> msg;
  msg->set_thing(42);
  mq.WriteMessage(msg);
  logger.AddQueue("testqueue", &mq);

  logger.Update();
}

TEST(Logger, LogsManyMessages) {
  std::unique_ptr<muan::logging::MockFileWriter> writer = std::make_unique<muan::logging::MockFileWriter>();

  EXPECT_CALL(*writer, WriteLine("testqueue.csv", "42")).Times(42);

  Logger logger(std::move(writer));

  MessageQueue<muan::proto::StackProto<logging_test::Test1, 256>, 100> mq;
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

  EXPECT_CALL(*writer, WriteLine("testqueue1.csv", "42")).Times(1);
  EXPECT_CALL(*writer, WriteLine("testqueue2.csv", "42")).Times(1);

  Logger logger(std::move(writer));

  MessageQueue<muan::proto::StackProto<logging_test::Test1, 256>, 100> mq1;
  muan::proto::StackProto<logging_test::Test1, 256> msg1;
  msg1->set_thing(42);
  mq1.WriteMessage(msg1);
  logger.AddQueue("testqueue1", &mq1);

  MessageQueue<muan::proto::StackProto<logging_test::Test1, 256>, 100> mq2;
  muan::proto::StackProto<logging_test::Test1, 256> msg2;
  msg2->set_thing(42);
  mq2.WriteMessage(msg2);
  logger.AddQueue("testqueue2", &mq2);

  logger.Update();
}

TEST(Logger, LogsManyMessagesPerTick) {
  std::unique_ptr<muan::logging::MockFileWriter> writer = std::make_unique<muan::logging::MockFileWriter>();

  EXPECT_CALL(*writer, WriteLine("testqueue.csv", "42")).Times(10000);

  Logger logger(std::move(writer));

  MessageQueue<muan::proto::StackProto<logging_test::Test1, 256>, 20000> mq;
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

  MessageQueue<muan::proto::StackProto<logging_test::Test1, 256>, 10> mq1;
  logger.AddQueue("testqueue", &mq1);

  MessageQueue<muan::proto::StackProto<logging_test::Test1, 256>, 25> mq2;
  ASSERT_DEATH(logger.AddQueue("testqueue", &mq2), "with same name \"testqueue\"");
}

TEST(Logger, TextLogger) {
  aos::time::Time::EnableMockTime();
  std::unique_ptr<muan::logging::MockFileWriter> writer = std::make_unique<muan::logging::MockFileWriter>();

  EXPECT_CALL(*writer, WriteLine("name.log", "1000,test")).Times(1);

  Logger logger(std::move(writer));
  auto textlog = logger.MakeTextLogger("name");
  aos::time::Time::SetMockTime(aos::time::Time::InSeconds(1));
  textlog("test");
  logger.Update();
}
}
}
