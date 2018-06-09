#ifndef MUAN_PROTO_STACK_PROTO_H_
#define MUAN_PROTO_STACK_PROTO_H_

#include <array>
#include <atomic>
#include <cstdint>

#include "google/protobuf/arena.h"
#include "third_party/aos/common/die.h"
#include "third_party/aos/common/time.h"

namespace muan {
namespace proto {

void* ProtoFailOnBlockAlloc(size_t);

void ProtoFailOnBlockFree(void*, size_t);

/*
 * A stack-allocated arena for protobuf messages. This class supports allocating
 * a single protocol buffer per arena. You can access the protobuf with pointer
 * syntax(operator->) or via get(). StackProto is templated with <T, size>,
 * where T is the typename of a protobuf message type and size is the size of
 * the buffer - you have to know how big the message you're creating could
 * possibly be.
 * Note: If you try to make a message bigger than the max, the program will die
 * Note: Strings will be allocated on the heap instead of in the arena, so avoid
 * them.
 * Example:
 *  StackProto<ProtoClass, 100> proto;
 *  proto->set_int_field(x);
 *  proto->set_bool_field(true);
 *  proto->set_string_field("Test!");
 *  ...
 *  StackProto<ProtoClass, 100> copy_a = proto;
 *  use(copy_a->int_field());
 *  ...
 *  StackProto<ProtoClass, 10> copy_b = proto;
 *  // Dies with "Buffer not big enough for proto!"
 */
template <typename T, std::size_t size>
class StackProto {
 public:
  using ProtoType = T;

  StackProto() : arena_(GetOptions()) {
    proto_message_ = google::protobuf::Arena::CreateMessage<T>(&arena_);
  }
  virtual ~StackProto() = default;

  // Allocate from a StackProto of a different size.
  template <std::size_t other_size>
  StackProto(const StackProto<T, other_size>& copy_from)
      : arena_{GetOptions()} {
    proto_message_ = google::protobuf::Arena::CreateMessage<T>(&arena_);
    proto_message_->CopyFrom(*copy_from.get());
  }

  // Allocate from a StackProto of the same size. This has to be templated
  // separately (I think because of how C++ recognizes copy constructors?)
  StackProto(const StackProto<T, size>& copy_from) : arena_{GetOptions()} {
    proto_message_ = google::protobuf::Arena::CreateMessage<T>(&arena_);
    proto_message_->CopyFrom(*copy_from.get());
  }

  const StackProto& operator=(const StackProto& copy_from) {
    // If we are copying, which creates a new message, we can discard the
    // contents of the old message to free up space
    arena_.Reset();

    proto_message_ = google::protobuf::Arena::CreateMessage<T>(&arena_);
    proto_message_->CopyFrom(*copy_from.get());
    return *this;
  }

  // Access to the proto
  T* operator->() { return proto_message_; }
  const T* operator->() const { return proto_message_; }
  T* get() { return proto_message_; }
  const T* get() const { return proto_message_; }

  // Reset the underlying arena. This will reset the protobuf message to its
  // default (empty) state.
  void Reset() {
    arena_.Reset();
    proto_message_ = google::protobuf::Arena::CreateMessage<T>(&arena_);
  }

 private:
  // Gets the options for the wrapped arena
  google::protobuf::ArenaOptions GetOptions() {
    google::protobuf::ArenaOptions options;

    // It's only allowed to use the preallocated buffer
    options.initial_block = &buffer_[0];
    options.initial_block_size = size;

    // Don't even think about malloc'ing
    options.start_block_size = 0;
    options.max_block_size = 0;

    // Fail instead of allocating more memory
    options.block_alloc = &ProtoFailOnBlockAlloc;
    options.block_dealloc = &ProtoFailOnBlockFree;

    return options;
  }

  // The buffer for the arena to use
  std::array<char, size> buffer_;

  google::protobuf::Arena arena_;

  // The message we're wrapping
  T* proto_message_{nullptr};
};

static const std::atomic<int64_t> start_time{
    std::chrono::duration_cast<std::chrono::milliseconds>(
        aos::monotonic_clock::now() - aos::monotonic_clock::epoch())
        .count()};

// This is a catch-all function overload, essentially if the other
// WriteTimestamp function doesn't compile the
// compiler will select this function which does nothing.
inline void WriteTimestamp(...) {}

// If the timestamp exists in the message that we're writing to a queue, this
// function is called. If it fails
// to compile, then the compiler will fall back to the other WriteTimestamp
// function. The decltype is there to
// check if the timestamp exists, and if it doesn't it fails to compile.
template <typename T>
auto WriteTimestamp(T* message)
    -> decltype((*message)->set_timestamp(0), void()) {
  (*message)->set_timestamp(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          aos::monotonic_clock::now() - aos::monotonic_clock::epoch())
          .count() -
      (aos::time::UsingMockTime() ? start_time.load() : 0));
}

// This secondary WriteTimestamp function makes it work if the message getting
// passed in is a pointer to a
// normal proto instead of a stack proto
template <typename T>
auto WriteTimestamp(T* message) -> decltype(message->set_timestamp(0), void()) {
  message->set_timestamp(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          aos::monotonic_clock::now() - aos::monotonic_clock::epoch())
          .count() -
      (aos::time::UsingMockTime() ? start_time.load() : 0));
}

}  // namespace proto
}  // namespace muan

#endif  // MUAN_PROTO_STACK_PROTO_H_
