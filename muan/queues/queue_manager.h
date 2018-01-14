#ifndef MUAN_QUEUES_QUEUE_MANAGER_H_
#define MUAN_QUEUES_QUEUE_MANAGER_H_

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <string>

#include "muan/logging/logger.h"
#include "muan/queues/message_queue.h"
#include "muan/utils/string_utils.h"
#include "muan/webdash/server.h"

namespace muan {

namespace queues {

void ResetAllQueues();

extern std::unique_ptr<logging::Logger> logger;
extern std::unique_ptr<webdash::WebDashRunner> webdash;

void Start();

template <typename T>
class QueueManager {
 public:
  // Fetch the queue of type T with the specified name, or create a new one with the given size.
  static MessageQueue<T>* Fetch(const char* key = "", int size = 200);

 private:
  static aos::Mutex all_queues_lock_;
  static std::unordered_map<std::string, MessageQueue<T>> all_queues_;
};

template <typename T>
aos::Mutex QueueManager<T>::all_queues_lock_;

template <typename T>
std::unordered_map<std::string, MessageQueue<T>> QueueManager<T>::all_queues_;

// A bunch of SFINAE tricks
// Add the proto to the webdash, if it's actually a proto
template <typename T>
void AddProtoQueueWebdash(...) {}
template <typename T>
auto AddProtoQueueWebdash(const char* str, MessageQueue<T>* queue)
    -> decltype(typename T::ProtoType(), void()) {
  if (webdash) {
    webdash->AddQueue(str, queue);
  }
}

// Add the proto to the logger, if it's actually a proto
template <typename T>
void AddProtoQueueLogger(...) {}
template <typename T>
auto AddProtoQueueLogger(const char* str, MessageQueue<T>* queue)
    -> decltype(typename T::ProtoType(), void()) {
  if (logger) {
    logger->AddQueue(str, queue);
  }
}

// Get the underlying type name if it's a StackProto
template <typename T>
const char* UnderlyingTypeName(...) {
  return typeid(T).name();
}
template <typename T>
auto UnderlyingTypeName(std::uint64_t) -> decltype(typename T::ProtoType(), (const char*)nullptr) {
  return UnderlyingTypeName<typename T::ProtoType>();
}
// No more SFINAE!

extern std::vector<GenericQueue*> all_queues_all_types;
extern aos::Mutex all_queues_all_types_lock;

extern "C" char* __cxa_demangle(const char* mangled_name, char* buf, size_t* n, int* status);

template <typename T>
MessageQueue<T>* QueueManager<T>::Fetch(const char* key, int size) {
  aos::MutexLocker locker_{&all_queues_lock_};

  // Try to find the right queue
  auto it = all_queues_.find(key);

  // If we can't find it, it hasn't been created yet!
  if (it == all_queues_.end()) {
    auto insert_result = all_queues_.emplace(key, MessageQueue<T>(size));

    // Check that the insert succeeded
    if (!insert_result.second) {
      return nullptr;
    }

    it = insert_result.first;

    char typename_buffer[1024];
    char filename_buffer[1024];
    size_t num_bytes = 1024;
    int status;

    __cxa_demangle(UnderlyingTypeName<T>(0), &typename_buffer[0], &num_bytes, &status);

    num_bytes = std::strlen(typename_buffer);

    // Find the last colon in the unmangled typename to find the start index of
    // the unqualified name
    size_t last_colon_idx = -1;
    for (size_t idx = 0; idx < num_bytes; idx++) {
      if (typename_buffer[idx] == ':') {
        last_colon_idx = idx;
      }
    }

    size_t idx = muan::utils::CamelToSnake(&typename_buffer[last_colon_idx + 1], num_bytes,
                                           &filename_buffer[0], 1024 - 5);

    size_t key_len = strlen(key);
    if (key_len > 0) {
      filename_buffer[idx] = '_';
      idx++;
    }

    std::strncpy(filename_buffer + idx, key, key_len);
    idx += key_len;

    // Add to webdash and logger
    AddProtoQueueWebdash<T>(filename_buffer, &(it->second));
    AddProtoQueueLogger<T>(filename_buffer, &(it->second));

    aos::MutexLocker locker{&all_queues_all_types_lock};
    all_queues_all_types.push_back(&(it->second));
  }

  // Return a pointer to the queue we pulled from the map
  return &(it->second);
}

}  // namespace queues

}  // namespace muan

#endif  // MUAN_QUEUES_QUEUE_MANAGER_H_
