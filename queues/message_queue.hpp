#ifndef QUEUES_MESSAGE_QUEUE_HPP_
#define QUEUES_MESSAGE_QUEUE_HPP_

#include "message_queue.h"

namespace muan {

namespace queues {

template <typename T, uint32_t size>
MessageQueue<T, size>::MessageQueue(MessageQueue<T, size>&& move_from)
    : messages_(move_from.messages_),
      back_(static_cast<uint32_t>(move_from.back_)) {}

template <typename T, uint32_t size>
void MessageQueue<T, size>::WriteMessage(const T& message) {
  // Push messages into the back
  uint32_t position = back_++;
  messages_[position % size] = message;
}

template <typename T, uint32_t size>
std::experimental::optional<T> MessageQueue<T, size>::NextMessage(
    uint32_t& next) {
  if (next >= back_) {
    next = back_;
    return std::experimental::nullopt;
  } else if (next < front()) {
    next = front();
  }
  auto current = next++;
  return messages_[current % size];
}

template <typename T, uint32_t size>
typename MessageQueue<T, size>::QueueReader
MessageQueue<T, size>::MakeReader() {
  return MessageQueue<T, size>::QueueReader{*this};
}

template <typename T, uint32_t size>
uint32_t MessageQueue<T, size>::front() {
  return std::max<uint32_t>(back_, size) - size;
}

template <typename T, uint32_t size>
MessageQueue<T, size>::QueueReader::QueueReader(
    MessageQueue<T, size>::QueueReader&& move_from)
    : queue_{move_from.queue_} {
  next_message_ = std::move(move_from.next_message_);
}

template <typename T, uint32_t size>
MessageQueue<T, size>::QueueReader::QueueReader(MessageQueue<T, size>& queue)
    : queue_(queue) {
  next_message_ = queue_.front();
}

template <typename T, uint32_t size>
std::experimental::optional<T>
MessageQueue<T, size>::QueueReader::ReadMessage() {
  return queue_.NextMessage(next_message_);
}

} /* queues */

} /* muan */

#endif /* QUEUES_MESSAGE_QUEUE_HPP_ */
