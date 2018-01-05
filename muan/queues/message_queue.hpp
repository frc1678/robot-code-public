#ifndef MUAN_QUEUES_MESSAGE_QUEUE_HPP_
#define MUAN_QUEUES_MESSAGE_QUEUE_HPP_

#include "muan/queues/message_queue.h"

#include <algorithm>
#include <utility>

#include "muan/utils/hash.h"

namespace muan {

namespace queues {

template <typename T>
MessageQueue<T>::MessageQueue(uint32_t size) : messages_(new T[size]), size_(size) {}

template <typename T>
MessageQueue<T>::MessageQueue(MessageQueue<T>&& move_from) noexcept
    : messages_(::std::move(move_from.messages_)),
      size_(move_from.size_),
      back_(move_from.back_) {}

template <typename T>
void MessageQueue<T>::WriteMessage(const T& message) {
  aos::MutexLocker locker_{&queue_lock_};
  // Push messages into the back
  messages_[back_ % size_] = message;

  // If the message contains a timestamp, add it to the message
  muan::proto::WriteTimestamp(&messages_[back_ % size_]);

  back_++;
}

template <typename T>
void MessageQueue<T>::Reset() {
  back_ = 0;
}

template <typename T>
bool MessageQueue<T>::NextMessage(T* out, uint64_t* next) const {
  aos::MutexLocker locker_{&queue_lock_};

  // Make sure the reader's index is within the bounds of still-valid messages,
  // and if it is at the end of the queue return nullopt.
  if (*next >= back_) {
    *next = back_;
    return false;
  }

  if (*next < front()) {
    *next = front();
  }

  auto current = *next;
  (*next)++;

  if (out) {
    *out = messages_[current % size_];
  }

  return true;
}

template <typename T>
uint64_t MessageQueue<T>::coerce_valid_message_index(uint64_t current_message_index) const {
  aos::MutexLocker locker_{&queue_lock_};

  uint64_t next = current_message_index;

  if (next >= back_) {
    next = back_;
  }

  if (next < front()) {
    next = front();
  }

  return next;
}

template <typename T>
std::experimental::optional<T> MessageQueue<T>::ReadLastMessage() const {
  T message;
  if (ReadLastMessage(&message)) {
    return message;
  } else {
    return std::experimental::nullopt;
  }
}

template <typename T>
bool MessageQueue<T>::ReadLastMessage(T* out) const {
  aos::MutexLocker locker_{&queue_lock_};

  if (back_ == 0) {
    // Nothing has ever been written to the queue!
    return false;
  }

  if (out) {
    *out = messages_[(back_ - 1) % size_];
  }
  return true;
}

template <typename T>
typename MessageQueue<T>::QueueReader MessageQueue<T>::MakeReader() const {
  return MessageQueue<T>::QueueReader{*this};
}

template <typename T>
uint64_t MessageQueue<T>::front() const {
  return front(back_);
}

template <typename T>
uint64_t MessageQueue<T>::front(uint64_t back) const {
  return std::max<uint64_t>(back, size_) - size_;
}

template <typename T>
MessageQueue<T>::QueueReader::QueueReader(MessageQueue<T>::QueueReader&& move_from) noexcept
    : queue_{move_from.queue_},
      next_message_{move_from.next_message_} {}

template <typename T>
MessageQueue<T>::QueueReader::QueueReader(const MessageQueue<T>& queue) : queue_(queue) {
  next_message_ = queue_.front();
}

template <typename T>
std::experimental::optional<T> MessageQueue<T>::QueueReader::ReadMessage() {
  T message;
  if (ReadMessage(&message)) {
    return message;
  } else {
    return std::experimental::nullopt;
  }
}

template <typename T>
bool MessageQueue<T>::QueueReader::ReadMessage(T* out) {
  return queue_.NextMessage(out, &next_message_);
}

template <typename T>
std::experimental::optional<T> MessageQueue<T>::QueueReader::ReadLastMessage() {
  T message;
  if (ReadLastMessage(&message)) {
    return message;
  } else {
    return std::experimental::nullopt;
  }
}

template <typename T>
bool MessageQueue<T>::QueueReader::ReadLastMessage(T* out) {
  next_message_ = queue_.back_;
  return queue_.ReadLastMessage(out);
}

template <typename T>
uint64_t MessageQueue<T>::QueueReader::GetNextMessageIndex() const {
  return queue_.coerce_valid_message_index(next_message_);
}

template <typename T>
uint64_t MessageQueue<T>::QueueReader::GetNumMessagesSkipped() const {
  return queue_.coerce_valid_message_index(next_message_) - next_message_;
}

}  // namespace queues

}  // namespace muan

#endif  // MUAN_QUEUES_MESSAGE_QUEUE_HPP_
