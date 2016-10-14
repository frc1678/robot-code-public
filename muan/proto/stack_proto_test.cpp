#ifndef MUAN_PROTO_STACK_PROTO_TEST_H_
#define MUAN_PROTO_STACK_PROTO_TEST_H_

#include "muan/proto/stack_proto.h"
#include "muan/proto/test_proto.pb.h"
#include "muan/queues/message_queue.h"
#include "gtest/gtest.h"

TEST(StackProto, Construct) {
  // Can we construct a single instance?
  muan::proto::StackProto<TestProto, 128> proto_a;
  proto_a->set_test_uint(3);
  EXPECT_EQ(proto_a->test_uint(), 3);
}

TEST(StackProto, Copyable) {
  // Make sure it's copy assignable and constructable.
  muan::proto::StackProto<TestProto, 128> proto_a;
  proto_a->set_test_uint(3);
  muan::proto::StackProto<TestProto, 128> proto_b{proto_a}, proto_c;
  proto_c = proto_a;
  EXPECT_EQ(proto_b->test_uint(), 3);
  EXPECT_EQ(proto_c->test_uint(), 3);
}

TEST(StackProto, NoDanglingPointers) {
  // If a StackProto is copy-assigned from another StackProto, it should not
  // have any pointers to the buffer of the one it was assigned to.
  muan::proto::StackProto<TestProto, 128> proto_a;
  {
    muan::proto::StackProto<TestProto, 128> proto_b;
    proto_b->set_test_uint(3);
    proto_a = proto_b;
  }
  EXPECT_EQ(proto_a->test_uint(), 3);
}

TEST(StackProto, DiesWhenTooSmall) {
  // When the buffer is too small for the protobuf, everything should die
  // horribly (with an exception)
  muan::proto::StackProto<TestProto, 128> proto_a;
  using ShortStackTestProto = muan::proto::StackProto<TestProto, 32>;
  EXPECT_THROW({ ShortStackTestProto proto_b{proto_a}; }, std::exception)
      << "Should generate an exception when the buffer doesn't have enough "
         "room!";
}

TEST(StackProto, ResetsOnAssign) {
  // The StackProto should reset its memory pool whenever it is assigned to a
  // different StackProto, freeing room for the new buffer.
  muan::proto::StackProto<TestProto, 128> proto_a;
  using SortaShortStackTestProto = muan::proto::StackProto<TestProto, 72>;
  SortaShortStackTestProto proto_b{proto_a};
  EXPECT_NO_THROW({
    for (size_t i = 0; i < 10; i++) {
      proto_b = proto_a;
    }
  });
}

TEST(StackProto, Queueable) {
  // Make sure it works in a queue.
  muan::queues::MessageQueue<muan::proto::StackProto<TestProto, 128>, 100>
      test_queue;
  auto reader = test_queue.MakeReader();

  {
    muan::proto::StackProto<TestProto, 128> proto;
    proto->set_test_uint(10);
    test_queue.WriteMessage(proto);
  }

  auto proto = *reader.ReadMessage();
  EXPECT_EQ(proto->test_uint(), 10);
}

TEST(StackProto, Reset) {
  // Make sure calling Reset() resets the memory.
  muan::proto::StackProto<TestProto, 128> proto;
  EXPECT_FALSE(proto->has_test_uint());
  proto->set_test_uint(3);
  proto.Reset();
  EXPECT_FALSE(proto->has_test_uint());
}

#endif  // MUAN_PROTO_STACK_PROTO_H_
