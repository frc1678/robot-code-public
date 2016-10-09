#include "logger.hpp"
#include "filewriter.h"
#include "gmock/gmock.h"
#include "muan/logging/logger_test.pb.h"
#include "muan/units/units.h"
#include "muan/utils/timing_utils.h"
#include "gtest/gtest.h"

using muan::logging::Logger;
using muan::queues::MessageQueue;
using namespace ::testing;

class MockFileWriter : public muan::logging::FileWriter {
 public:
  MockFileWriter() : muan::logging::FileWriter("/"){};
  MOCK_METHOD2(WriteLine, void(std::string filename, std::string line));
};

TEST(Logger, LogsOneMessage) {
  std::shared_ptr<MockFileWriter> mock_writer = std::make_shared<MockFileWriter>();

  EXPECT_CALL(*mock_writer, WriteLine("testqueue.csv", "42"))
      .Times(1);

  std::shared_ptr<muan::logging::FileWriter> writer = mock_writer;
  Logger logger(writer);

  MessageQueue<logging_test::Test1, 100> mq;
  logging_test::Test1 msg;
  msg.set_thing(42);
  mq.WriteMessage(msg);
  auto mqr = mq.MakeReader();
  logger.AddQueue("testqueue", mqr);

  logger.Update(0.005);
}

TEST(Logger, LogsManyMessages) {
  std::shared_ptr<MockFileWriter> mock_writer = std::make_shared<MockFileWriter>();

  EXPECT_CALL(*mock_writer, WriteLine("testqueue.csv", "42"))
      .Times(42);

  std::shared_ptr<muan::logging::FileWriter> writer = mock_writer;
  Logger logger(writer);

  MessageQueue<logging_test::Test1, 100> mq;
  for (int i = 1; i <= 42; i++) {
    logging_test::Test1 msg;
    msg.set_thing(42);
    mq.WriteMessage(msg);
  }
  auto mqr = mq.MakeReader();
  logger.AddQueue("testqueue", mqr);

  logger.Update(0.005);
}

TEST(Logger, LogsMultipleQueues) {
  std::shared_ptr<MockFileWriter> mock_writer = std::make_shared<MockFileWriter>();

  EXPECT_CALL(*mock_writer, WriteLine("testqueue1.csv", "42"))
      .Times(1);
  EXPECT_CALL(*mock_writer, WriteLine("testqueue2.csv", "42"))
      .Times(1);

  std::shared_ptr<muan::logging::FileWriter> writer = mock_writer;
  Logger logger(writer);

  MessageQueue<logging_test::Test1, 100> mq1;
  logging_test::Test1 msg1;
  msg1.set_thing(42);
  mq1.WriteMessage(msg1);
  auto mqr1 = mq1.MakeReader();
  logger.AddQueue("testqueue1", mqr1);

  MessageQueue<logging_test::Test1, 100> mq2;
  logging_test::Test1 msg2;
  msg2.set_thing(42);
  mq2.WriteMessage(msg2);
  auto mqr2 = mq2.MakeReader();
  logger.AddQueue("testqueue2", mqr2);

  logger.Update(0.005);
}

TEST(Logger, LogsManyMessagesPerTick) {
  std::shared_ptr<MockFileWriter> mock_writer = std::make_shared<MockFileWriter>();

  EXPECT_CALL(*mock_writer, WriteLine("testqueue.csv", "42"))
      .Times(10000);

  std::shared_ptr<muan::logging::FileWriter> writer = mock_writer;
  Logger logger(writer);

  MessageQueue<logging_test::Test1, 20000> mq;
  auto mqr = mq.MakeReader();
  logger.AddQueue("testqueue", mqr);

  for (int n = 1; n <= 10; n++) {
    for (int i = 1; i <= 1000; i++) {
      logging_test::Test1 msg;
      msg.set_thing(42);
      mq.WriteMessage(msg);
    }
    logger.Update(0.005);
  }
}

TEST(Logger, RunsAsThread) {
  std::shared_ptr<MockFileWriter> mock_writer = std::make_shared<MockFileWriter>();

  EXPECT_CALL(*mock_writer, WriteLine("testqueue.csv", "42"))
      .Times(1);

  std::shared_ptr<muan::logging::FileWriter> writer = mock_writer;
  Logger logger(writer);

  MessageQueue<logging_test::Test1, 100> mq;
  logging_test::Test1 msg;
  msg.set_thing(42);
  mq.WriteMessage(msg);
  auto mqr = mq.MakeReader();
  logger.AddQueue("testqueue", mqr);

  logger.Start();
  muan::sleep_for(2 * muan::units::s);
  logger.Stop();
  muan::sleep_for(1 * muan::units::s);
}

TEST(Logger, TextLogger) {
  std::shared_ptr<MockFileWriter> mock_writer = std::make_shared<MockFileWriter>();
  std::shared_ptr<muan::logging::FileWriter> writer = mock_writer;

  EXPECT_CALL(*mock_writer, WriteLine("name.log", "test"))
      .Times(1);

  Logger logger(writer);
  auto textlog = logger.GetTextLogger("name");
  textlog("test");
  logger.Update(0.005);
}
