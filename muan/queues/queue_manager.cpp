#include "muan/queues/queue_manager.h"
#include <memory>
#include <thread>
#include <vector>

namespace muan {
namespace queues {

std::unique_ptr<logging::Logger> logger;
std::unique_ptr<webdash::WebDashRunner> webdash;

std::vector<GenericQueue*> all_queues_all_types;
aos::Mutex all_queues_all_types_lock;

void ResetAllQueues() {
  aos::MutexLocker locker{&all_queues_all_types_lock};
  for (GenericQueue* q : all_queues_all_types) {
    q->Reset();
  }
}

void Start() {
  logger = std::make_unique<logging::Logger>();
  std::thread logger_thread{std::ref(*logger)};
  logger_thread.detach();

  webdash = std::make_unique<webdash::WebDashRunner>();
  std::thread webdash_thread{std::ref(*webdash)};
  webdash_thread.detach();
}

}  // namespace queues
}  // namespace muan
