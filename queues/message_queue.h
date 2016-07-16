#ifndef MUAN_QUEUES_MESSAGE_QUEUE_H_
#define MUAN_QUEUES_MESSAGE_QUEUE_H_

#include "../utils/math_utils.h"
#include "optional/optional.hpp"
#include <array>
#include <atomic>
#include <cstdint>

namespace muan {

namespace queues {

template <typename T, uint32_t size = 100>
class MessageQueue {
 public:
  MessageQueue() = default;
  virtual ~MessageQueue() = default;

  // Enable moving, disable copying
  MessageQueue(MessageQueue&& move_from);
  MessageQueue& operator=(MessageQueue&& move_from) = default;
  MessageQueue(const MessageQueue&) = delete;
  const MessageQueue& operator=(const MessageQueue&) = delete;

  void WriteMessage(const T& message);

  class QueueReader {
   public:
    std::experimental::optional<T> ReadMessage();

    // Allow move constructor but not move assignment as it cannot be reassigned
    // to another queue
    QueueReader(QueueReader&& move_from);
    QueueReader& operator=(QueueReader&& move_from) = delete;

    // Prevent copying
    QueueReader(const QueueReader&) = delete;
    const QueueReader& operator=(const QueueReader&) = delete;

    virtual ~QueueReader() = default;

   private:
    QueueReader(MessageQueue& queue);

    MessageQueue& queue_;
    uint32_t next_message_;

    friend class MessageQueue;
  };

  QueueReader MakeReader();

 private:
  std::experimental::optional<T> NextMessage(uint32_t& next);

  // If the back wraps around and "catches up" with the front, drop the front
  // message and move the front forward
  uint32_t front();

  std::array<T, size> messages_;
  std::atomic<uint32_t> back_{0};
};

} /* queues */

} /* muan */

#include "message_queue.hpp"

#endif /* MUAN_QUEUES_MESSAGE_QUEUE_H_ */
