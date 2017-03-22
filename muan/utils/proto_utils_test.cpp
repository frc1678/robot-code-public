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

  {
    std::ostringstream ss;
    muan::util::ProtoToCsvHeader(p, ss);
    ASSERT_EQ(ss.str(), "test_string,test_uint,sub_message.id,sub_message.num,is_sane");
  }

  {
    std::ostringstream ss;
    muan::util::ProtoToCsv(p, ss);
    ASSERT_EQ(ss.str(), "Citrus Circuits,1678,100,3.14159,0");
  }
}

TEST(ProtoUtils, OptionalFields) {
  TestProto p;
  // Don't set any fields
  {
    std::ostringstream ss;
    muan::util::ProtoToCsvHeader(p, ss);
    ASSERT_EQ(ss.str(), "test_string,test_uint,sub_message.id,sub_message.num,is_sane");
  }

  {
    std::ostringstream ss;
    muan::util::ProtoToCsv(p, ss);
    ASSERT_EQ(ss.str(), ",0,0,0,0");
  }
}

TEST(ProtoUtils, Repeated) {
  // Test that we die on repeated fields
  DeathProto p;
  p.add_test_string("aaa");

  std::ostringstream ss;

  EXPECT_DEATH(muan::util::ProtoToCsvHeader(p, ss), "repeated message");
  EXPECT_DEATH(muan::util::ProtoToCsv(p, ss), "repeated message");
}

TEST(ProtoUtils, Enums) {
  EnumTest p;
  {
    std::ostringstream ss;
    muan::util::ProtoToCsv(p, ss);
    EXPECT_EQ(ss.str(), "0");
  }

  p.set_test_enum(EnumTest::SOME_OPTION);
  {
    std::ostringstream ss;
    muan::util::ProtoToCsv(p, ss);
    EXPECT_EQ(ss.str(), "1");
  }
}

TEST(ProtoUtils, OneOf) {
  OneofTest p;
  {
    std::ostringstream ss;
    muan::util::ProtoToCsvHeader(p, ss);
    EXPECT_EQ(ss.str(), "test_1.val,test_2.val");
  }

  {
    std::ostringstream ss;
    muan::util::ProtoToCsv(p, ss);
    EXPECT_EQ(ss.str(), "0,");
  }

  p.mutable_test_1()->set_val(1.0);

  {
    std::ostringstream ss;
    muan::util::ProtoToCsv(p, ss);
    EXPECT_EQ(ss.str(), "1,");
  }

  p.mutable_test_2()->set_val("test");

  {
    std::ostringstream ss;
    muan::util::ProtoToCsv(p, ss);
    EXPECT_EQ(ss.str(), "0,test");
  }
}

TEST(ProtoUtils, ProtoToJson) {
  TestProto p;
  p.set_test_string("Citrus Circuits");
  p.set_test_uint(1678);
  p.mutable_sub_message()->set_id(100);
  p.mutable_sub_message()->set_num(3.14159);
  p.set_is_sane(false);

  {
    std::ostringstream ss;
    muan::util::ProtoToJson(p, ss);
    ASSERT_EQ(ss.str(), "{\"testString\":\"Citrus Circuits\",\"testUint\":1678,\"subMessage\":{\"id\":100,\"num\":3.14159},\"isSane\":0}");  //NOLINT
  }
}
