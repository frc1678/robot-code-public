#include "generic_robot/queue_manager/queue_manager.h"
#include "gtest/gtest.h"

TEST(QueueManager, Initalizes) { generic_robot::QueueManager::GetInstance(); }

TEST(QueueManager, QueueWorks) {
  muan::proto::StackProto<PdpStatus, 512> p;
  generic_robot::QueueManager::GetInstance()->pdp_status_queue()->WriteMessage(
      p);

  auto pdp_status_reader = generic_robot::QueueManager::GetInstance()
                               ->pdp_status_queue()
                               ->MakeReader();

  // Test that it reads one message (the message we sent above), then that it
  // doesn't have any new messages (because we just read the only message).
  ASSERT_TRUE(pdp_status_reader.ReadMessage());
  ASSERT_FALSE(pdp_status_reader.ReadMessage());
}
