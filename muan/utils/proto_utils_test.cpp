#include "muan/utils/proto_utils.h"
#include <iostream>
#include "gtest/gtest.h"
#include "muan/utils/test_death_proto.pb.h"
#include "muan/utils/test_proto.pb.h"

TEST(ProtoUtils, ProtoToCSV) {
  TestProto p;
  p.set_test_string("Citrus Circuits");
  p.set_test_uint(1678);
  p.mutable_sub_message()->set_id(100);
  p.mutable_sub_message()->set_num(3.14159);
  p.set_is_sane(false);
  ASSERT_EQ(muan::util::ProtoToCSVHeader(p), "test_string,test_uint,sub_message.id,sub_message.num,is_sane");
  ASSERT_EQ(muan::util::ProtoToCSV(p), "Citrus Circuits,1678,100,3.141590,0");
}

TEST(ProtoUtils, Repeated) {
  // Test that we die on repeated fields
  TestDeathProto p;
  p.add_test_string("aaa");
  EXPECT_DEATH(muan::util::ProtoToCSV(p), "repeated message");
}
