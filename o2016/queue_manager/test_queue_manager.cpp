#include "gtest/gtest.h"
#include "o2016/queue_manager/queue_manager.h"

TEST(QueueManager, Initalizes) { o2016::QueueManager::GetInstance(); }

TEST(QueueManager, QueueWorks) {
  muan::proto::StackProto<PdpStatus, 1024> p;
  o2016::QueueManager::GetInstance().pdp_status_queue().WriteMessage(p);

  auto pdp_status_reader = o2016::QueueManager::GetInstance().pdp_status_queue().MakeReader();

  // Test that it reads one message (the message we sent above), then that it
  // doesn't have any new messages (because we just read the only message).
  ASSERT_TRUE(pdp_status_reader.ReadMessage());
  ASSERT_FALSE(pdp_status_reader.ReadMessage());
}
