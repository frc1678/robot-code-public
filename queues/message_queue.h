#ifndef MUAN_QUEUES_MESSAGE_QUEUE_H_
#define MUAN_QUEUES_MESSAGE_QUEUE_H_

#include "../utils/math_utils.h"
#include <array>
#include <atomic>
#include <cstdint>
#include <experimental/optional>

namespace muan {

namespace queues {

template <typename T, uint32_t size = 100>
class MessageQueue {
 public:
  MessageQueue() {}
  virtual ~MessageQueue() {}

  // Enable moving, disable copying
  MessageQueue(MessageQueue&& move_from)
      : messages_(move_from.messages_),
        back_(static_cast<uint32_t>(move_from.back_)) {}

  MessageQueue& operator=(MessageQueue&& move_from) = default;
  MessageQueue(const MessageQueue&) = delete;
  const MessageQueue& operator=(const MessageQueue&) = delete;

  void WriteMessage(const T& message) {
    // Push messages into the back
    uint32_t position = back_++;
    messages_[position % size] = message;
  }

  class QueueReader {
   public:
    std::experimental::optional<T> ReadMessage() {
      return queue_.NextMessage(next_message_);
    }

    // Allow move constructor but not move assignment as it cannot be reassigned
    // to another queue
    QueueReader(QueueReader&& move_from) : queue_{move_from.queue_} {
      next_message_ = std::move(move_from.next_message_);
    }
    QueueReader& operator=(QueueReader&& move_from) = delete;

    // Prevent copying
    QueueReader(const QueueReader&) = delete;
    const QueueReader& operator=(const QueueReader&) = delete;

    virtual ~QueueReader() {}

   private:
    QueueReader(MessageQueue& queue) : queue_(queue) {
      next_message_ = queue_.front_();
    }

    MessageQueue& queue_;
    uint32_t next_message_;

    friend class MessageQueue;
  };

  QueueReader MakeReader() { return QueueReader{*this}; }

 private:
  std::experimental::optional<T> NextMessage(uint32_t& next) {
    using namespace std::experimental;
    if (next >= back_) {
      next = back_;
      return std::experimental::nullopt;
    } else if (next < front_()) {
      next = front_();
    }
    auto current = next;
    next++;
    return messages_[current % size];
  }
  
  // If the back wraps around and "catches up" with the front, drop the front
  // message and move the front forward
  uint32_t front_() {
   if (back_ > size) {
    return back_ - size;
   }
   return 0;
  }
  std::array<T, size> messages_;
  std::atomic<uint32_t> back_{0};
};

} /* queues */

} /* muan */

#endif /* MUAN_QUEUES_MESSAGE_QUEUE_H_ */
