#include "queue_manager.h"

namespace o2016 {

QueueManager& QueueManager::GetInstance() {
  static QueueManager instance;
  return instance;
}

MessageQueue<muan::proto::StackProto<PdpStatus, 512>>& QueueManager::get_pdp_status_queue() {
  return pdp_status_queue_;
}

}
