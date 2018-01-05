#include "muan/queues/queue_manager.h"
#include <thread>
#include <vector>

namespace muan {
namespace queues {

#ifndef FRC1678_NO_QUEUE_LOGGING
logging::Logger logger;
#endif  // FRC1678_NO_QUEUE_LOGGING
webdash::WebDashRunner webdash;

std::vector<GenericQueue*> all_queues_all_types;
aos::Mutex all_queues_all_types_lock;

void ResetAllQueues() {
  aos::MutexLocker locker{&all_queues_all_types_lock};
  for (GenericQueue* q : all_queues_all_types) {
    q->Reset();
  }
}

struct DetachThreadsOnInit {
  DetachThreadsOnInit() {
#ifndef FRC1678_NO_QUEUE_LOGGING
    std::thread logger_thread{std::ref(logger)};
    logger_thread.detach();
#endif  // FRC1678_NO_QUEUE_LOGGING
    std::thread webdash_thread{std::ref(webdash)};
    webdash_thread.detach();
  }

  static DetachThreadsOnInit instance;
};

DetachThreadsOnInit DetachThreadsOnInit::instance;

}  // namespace queues
}  // namespace muan
