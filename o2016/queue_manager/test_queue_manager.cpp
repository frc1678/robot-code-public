#include "queue_manager.h"
#include "gtest/gtest.h"

TEST(QueueManager, Initalizes) {
  o2016::QueueManager::GetInstance();
}

TEST(QueueManager, QueueWorks) {
  muan::proto::StackProto<PdpStatus, 512> p;
  o2016::QueueManager::GetInstance().pdp_status_queue().WriteMessage(p);

  auto pdp_status_reader = o2016::QueueManager::GetInstance().pdp_status_queue().MakeReader();

  ASSERT_TRUE(pdp_status_reader.ReadMessage());
  ASSERT_FALSE(pdp_status_reader.ReadMessage());
}
