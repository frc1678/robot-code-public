#include "muan/utils/proto_utils.h"
#include "gtest/gtest.h"
#include "muan/utils/test_proto.pb.h"

TEST(ProtoUtils, ProtoToCsv) {
  TestProto p;
  p.set_test_string("Citrus Circuits");
  p.set_test_uint(1678);
  p.mutable_sub_message()->set_id(100);
  p.mutable_sub_message()->set_num(3.14159);
  p.set_is_sane(false);
  ASSERT_EQ(muan::util::ProtoToCsvHeader(p), "test_string,test_uint,sub_message.id,sub_message.num,is_sane");
  ASSERT_EQ(muan::util::ProtoToCsv(p), "Citrus Circuits,1678,100,3.14159,0");
}

TEST(ProtoUtils, OptionalFields) {
  TestProto p;
  // Don't set any fields
  ASSERT_EQ(muan::util::ProtoToCsvHeader(p), "test_string,test_uint,sub_message.id,sub_message.num,is_sane");
  ASSERT_EQ(muan::util::ProtoToCsv(p), ",0,0,0,0");
}

TEST(ProtoUtils, Repeated) {
  // Test that we die on repeated fields
  DeathProto p;
  p.add_test_string("aaa");
  EXPECT_DEATH(muan::util::ProtoToCsv(p), "repeated message");
}

TEST(ProtoUtils, Enums) {
  EnumTest p;
  EXPECT_EQ(muan::util::ProtoToCsv(p), "0");
  p.set_test_enum(EnumTest::SOME_OPTION);
  EXPECT_EQ(muan::util::ProtoToCsv(p), "1");
}

TEST(ProtoUtils, OneOf) {
  OneofTest p;
  EXPECT_EQ(muan::util::ProtoToCsvHeader(p), "test_1.val,test_2.val");
  EXPECT_EQ(muan::util::ProtoToCsv(p), "0,");
  p.mutable_test_1()->set_val(1.0);
  EXPECT_EQ(muan::util::ProtoToCsv(p), "1,");
  p.mutable_test_2()->set_val("test");
  EXPECT_EQ(muan::util::ProtoToCsv(p), "0,test");
}
