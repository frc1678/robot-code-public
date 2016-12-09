#ifndef MUAN_QUEUES_MESSAGE_QUEUE_HPP_
#define MUAN_QUEUES_MESSAGE_QUEUE_HPP_

#include "message_queue.h"

namespace muan {

namespace queues {

template <typename T, uint64_t size>
MessageQueue<T, size>::MessageQueue(MessageQueue<T, size>&& move_from) noexcept
    : messages_(move_from.messages_),
      back_(move_from.back_) {}

template <typename T, uint64_t size>
void MessageQueue<T, size>::WriteMessage(const T& message) {
  aos::MutexLocker locker_{&queue_lock_};
  // Push messages into the back
  messages_[back_ % size] = message;

  back_++;
}

template <typename T, uint64_t size>
void MessageQueue<T, size>::Reset() {
  back_ = 0;
}

template <typename T, uint64_t size>
std::experimental::optional<T> MessageQueue<T, size>::NextMessage(
    uint64_t& next) const {
  aos::MutexLocker locker_{&queue_lock_};

  // Make sure the reader's index is within the bounds of still-valid messages,
  // and if it is at the end of the queue return nullopt.
  if (next >= back_) {
    next = back_;
    return std::experimental::nullopt;
  }

  if (next < front()) {
    next = front();
  }

  auto current = next;
  next++;
  return messages_[current % size];
}

template <typename T, uint64_t size>
std::experimental::optional<T> MessageQueue<T, size>::ReadLastMessage() const {
  aos::MutexLocker locker_{&queue_lock_};

  if (back_ == 0) {
    // Nothing has ever been written to the queue! Return nullopt!
    return std::experimental::nullopt;
  } else {
    return messages_[(back_ - 1) % size];
  }
}

template <typename T, uint64_t size>
typename MessageQueue<T, size>::QueueReader MessageQueue<T, size>::MakeReader()
    const {
  return MessageQueue<T, size>::QueueReader{*this};
}

template <typename T, uint64_t size>
uint64_t MessageQueue<T, size>::front() const {
  return front(back_);
}

template <typename T, uint64_t size>
uint64_t MessageQueue<T, size>::front(uint64_t back) const {
  return std::max<uint64_t>(back, size) - size;
}

template <typename T, uint64_t size>
MessageQueue<T, size>::QueueReader::QueueReader(
    MessageQueue<T, size>::QueueReader&& move_from) noexcept
    : queue_{move_from.queue_},
      next_message_{move_from.next_message_} {}

template <typename T, uint64_t size>
MessageQueue<T, size>::QueueReader::QueueReader(
    const MessageQueue<T, size>& queue)
    : queue_(queue) {
  next_message_ = queue_.front();
}

template <typename T, uint64_t size>
std::experimental::optional<T>
MessageQueue<T, size>::QueueReader::ReadMessage() {
  return queue_.NextMessage(next_message_);
}

template <typename T, uint64_t size>
std::experimental::optional<T>
MessageQueue<T, size>::QueueReader::ReadLastMessage() {
  next_message_ = queue_.back_;
  return queue_.ReadLastMessage();
}

}  // namespace queues

}  // namespace muan

#endif /* MUAN_QUEUES_MESSAGE_QUEUE_HPP_ */
