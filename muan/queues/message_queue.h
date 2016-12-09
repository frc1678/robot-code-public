#ifndef MUAN_QUEUES_MESSAGE_QUEUE_H_
#define MUAN_QUEUES_MESSAGE_QUEUE_H_

#include "muan/utils/math_utils.h"
#include "third_party/aos/common/mutex.h"
#include "third_party/optional/optional.hpp"
#include <array>
#include <cstdint>

namespace muan {

namespace queues {

/*
 * A thread-safe FILO buffer to be used as a message-passing system for transfer
 * of data between threads. A MessageQueue can only be written to by a single
 * thread at a time, but it can create multiple QueueReaders to allow access to
 * the queue. As long as the reader is able to read as least as quickly as
 * messages are being written, it will not skip data.
 * Example:
 *    MessageQueue<DrivetrainState> state_queue;
 *    auto state_reader = state_queue.MakeReader();
 *    ...
 *    state_queue.WriteMessage(state);
 *    ...
 *    auto maybe_state = state_reader.ReadMessage();
 *    if (maybe_state) {
 *      // Do something with the state here
 *    }
 * Note: All variables representing positions in the queue refer to the absolute
 * position - in other words, a position that increments for each new message
 * and never decreases. This allows the queue to understand the ordering of
 * positions much more easily.
 */
template <typename T, uint64_t size = 100>
class MessageQueue {
 public:
  MessageQueue() = default;
  virtual ~MessageQueue() = default;

  // Enables moving, disables copying.
  MessageQueue(MessageQueue&& move_from) noexcept;
  MessageQueue& operator=(MessageQueue&& move_from) = default;  // NOLINT
  MessageQueue(const MessageQueue&) = delete;
  const MessageQueue& operator=(const MessageQueue&) = delete;

  // Writes a message to the queue. If the queue is full, it will overwrite the
  // oldest message.
  void WriteMessage(const T& message);

  std::experimental::optional<T> ReadLastMessage();

  // Reset the queue to the empty state
  void Reset();

  class QueueReader {
   public:
    // Reads a single message from the queue or nullopt if there are no new
    // messages.
    std::experimental::optional<T> ReadMessage();

    // Reads the last message from the queue or nullopt if there are no new
    // messages. Also marks all previous messages as read.
    std::experimental::optional<T> ReadLastMessage();

    // Allows move construction but not move assignment - it doesn't really make
    // sense to assign a queue to another queue.
    QueueReader(QueueReader&& move_from) noexcept;
    QueueReader& operator=(QueueReader&& move_from) = delete;

    // Prevents copying.
    QueueReader(const QueueReader&) = delete;
    const QueueReader& operator=(const QueueReader&) = delete;

    virtual ~QueueReader() = default;

   private:
    // Creates a QueueReader from a MessageQueue.
    explicit QueueReader(const MessageQueue& queue);

    const MessageQueue& queue_;
    uint64_t next_message_;

    friend class MessageQueue;
  };

  // Creates a QueueReader for this queue. The QueueReader's first message will
  // be the oldest message still present in the queue.
  QueueReader MakeReader() const;

 private:
  // Gets the next message (or nullopt if all messages have been read) from the
  // position passed in. The parameter's value will be changed to the position
  // of the next valid message.
  std::experimental::optional<T> NextMessage(uint64_t& next) const;

  // Gets the most recent message from the queue.
  std::experimental::optional<T> LastMessage() const;

  // Gets the "front" (the oldest messages still kept) of the circular buffer,
  // either from the current value of _back or from a known value of back.
  // Note: before accessing front(), the caller should hold the queue_lock_, as
  // back_ is not atomic. However, front(uint64_t) can be used without a lock
  // because it uses some existing value of back_.
  uint64_t front() const;
  uint64_t front(uint64_t back) const;

  // A buffer and an index to implement a circular buffer. back_ is not in mod n
  // - that is, it keeps incrementing and never jumps back around to 0.
  std::array<T, size> messages_;
  uint64_t back_{0};

  // A lock for the entire queue. This mutex is used to protect access to back_
  // and messages_ to allow access by multiple threads at the same time.
  mutable aos::Mutex queue_lock_;
};

}  // namespace queues

}  // namespace muan

#include "message_queue.hpp"

#endif /* MUAN_QUEUES_MESSAGE_QUEUE_H_ */
