#ifndef MUAN_QUEUES_MESSAGE_QUEUE_H_
#define MUAN_QUEUES_MESSAGE_QUEUE_H_

#include "../utils/math_utils.h"
#include "optional/optional.hpp"
#include <array>
#include <atomic>
#include <cstdint>

namespace muan {

namespace queues {

/*
 * A thread-safe FILO buffer to be used as a message-passing system for transfer
 * of data between threads. A MessageQueue can create Readers to allow access to
 * the queue. As long as the reader approximately keeps pace with the speed at
 * which data is written to the queue, data will remain valid.
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
template <typename T, uint32_t size = 100>
class MessageQueue {
 public:
  MessageQueue() = default;
  virtual ~MessageQueue() = default;

  // Enables moving, disables copying.
  MessageQueue(MessageQueue&& move_from);
  MessageQueue& operator=(MessageQueue&& move_from) = default;
  MessageQueue(const MessageQueue&) = delete;
  const MessageQueue& operator=(const MessageQueue&) = delete;

  // Writes a message to the queue. If the queue is full, it will overwrite the
  // oldest message.
  void WriteMessage(const T& message);

  class QueueReader {
   public:
    // Reads a single message from the queue or nullopt if there are no new
    // messages.
    std::experimental::optional<T> ReadMessage();

    // Allows move construction but not move assignment - it doesn't really make
    // sense to assign a queue to another queue.
    QueueReader(QueueReader&& move_from);
    QueueReader& operator=(QueueReader&& move_from) = delete;

    // Prevents copying.
    QueueReader(const QueueReader&) = delete;
    const QueueReader& operator=(const QueueReader&) = delete;

    virtual ~QueueReader() = default;

   private:
    // Creates a QueueReader from a MessageQueue.
    QueueReader(MessageQueue& queue);

    MessageQueue& queue_;
    uint32_t next_message_;

    friend class MessageQueue;
  };

  // Creates a QueueReader for this queue. The QueueReader's first message will
  // be the oldest message still present in the queue.
  QueueReader MakeReader();

 private:
  // Gets the next message (or nullopt if all messages have been read) from the
  // position passed in. The parameter's value will be changed to the position
  // of the next valid message.
  std::experimental::optional<T> NextMessage(uint32_t& next);

  uint32_t front() const;

  std::array<T, size> messages_;
  std::atomic<uint32_t> back_{0};
};

} /* queues */

} /* muan */

#include "message_queue.hpp"

#endif /* MUAN_QUEUES_MESSAGE_QUEUE_H_ */
