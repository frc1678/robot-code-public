#ifndef MUAN_QUEUES_MESSAGE_QUEUE_HPP_
#define MUAN_QUEUES_MESSAGE_QUEUE_HPP_

#include "message_queue.h"

namespace muan {

namespace queues {

template <typename T, uint32_t size>
MessageQueue<T, size>::MessageQueue(MessageQueue<T, size>&& move_from) noexcept
    : messages_(move_from.messages_),
      back_(static_cast<uint32_t>(move_from.back_)) {}

template <typename T, uint32_t size>
void MessageQueue<T, size>::WriteMessage(const T& message) {
  // Push messages into the back
  messages_[back_ % size] = message;

  // Increment the count after the emplacement into the array. This fixes the
  // synchronization issue in which a reader interrupts the writer before it has
  // finished copying the message into the buffer, but does not allow for
  // multiple writers. At this point in time (July 2016) we do not need
  // multiple-writer queues, but if we do need that feature a possible solution
  // would be to increment a separate atomic index back_writer_ before the
  // copying and increment back_ after the copying.
  back_++;
}

template <typename T, uint32_t size>
std::experimental::optional<T> MessageQueue<T, size>::NextMessage(
    uint32_t& next) const {
  // Capture the values of back_ and front() so that they cannot be changed
  // during the execution of this function.
  uint32_t back_capture = back_;
  uint32_t front_capture = front(back_capture);

  // Make sure the reader's index is within the bounds of still-valid messages,
  // and if it is at the end of the queue return nullopt.
  if (next >= back_capture) {
    next = back_capture;
    return std::experimental::nullopt;
  }

  if (next < front_capture) {
    next = front_capture;
  }

  auto current = next++;
  return messages_[current % size];
}

template <typename T, uint32_t size>
typename MessageQueue<T, size>::QueueReader MessageQueue<T, size>::MakeReader()
    const {
  return MessageQueue<T, size>::QueueReader{*this};
}

template <typename T, uint32_t size>
uint32_t MessageQueue<T, size>::front() const {
  return front(back_);
}

template <typename T, uint32_t size>
uint32_t MessageQueue<T, size>::front(uint32_t back) const {
  return std::max<uint32_t>(back, size) - size;
}

template <typename T, uint32_t size>
MessageQueue<T, size>::QueueReader::QueueReader(
    MessageQueue<T, size>::QueueReader&& move_from) noexcept
    : queue_{move_from.queue_} {
  next_message_ = std::move(move_from.next_message_);
}

template <typename T, uint32_t size>
MessageQueue<T, size>::QueueReader::QueueReader(
    const MessageQueue<T, size>& queue)
    : queue_(queue) {
  next_message_ = queue_.front();
}

template <typename T, uint32_t size>
std::experimental::optional<T>
MessageQueue<T, size>::QueueReader::ReadMessage() {
  return queue_.NextMessage(next_message_);
}

}  // namespace queues

}  // namespace muan

#endif /* MUAN_QUEUES_MESSAGE_QUEUE_HPP_ */
