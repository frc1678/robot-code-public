#ifndef MUAN_UTILS_PROTO_UTILS_H_
#define MUAN_UTILS_PROTO_UTILS_H_

#include <iostream>
#include "google/protobuf/message.h"
#include "third_party/aos/common/die.h"

namespace muan {
namespace util {

// Writes the contents of this message, converted into CSV format, to an
// ostream. The output will be ordered
// by field number, with empty optional values represented as the default value
// for that data type (i.e. empty
// for a string, 0 for numerics). Submessage values will be written in field
// order number as well, with all
// data points for a submessage inserted into the line in the position of the
// submessage field. Booleans will
// be written as 1/0 and enums as numeric values.
//
// For example, TestProto from test_proto.proto will default to:
// ,0,0,0,0
// And a sample populated value might look like:
// Citrus,1678,1,3.14,1
void ProtoToCsv(const google::protobuf::Message& message,
                std::ostream& serialize);

// Writes the header for this message, in CSV format, to an ostream. The output
// will be ordered by field
// number, and only the name of each field will be written out. All fields will
// be written, including all
// possible values of oneof fields. Submessage fields will be separated by a
// dot.
//
// For example, running this on a TestProto from test_proto.proto will write:
// test_string,test_uint,sub_message.id,sub_message.num,is_sane
void ProtoToCsvHeader(const google::protobuf::Message& message,
                      std::ostream& serialize);

void ProtoToJson(const google::protobuf::Message& message,
                 std::ostream& serialize);

}  // namespace util
}  // namespace muan

#endif  // MUAN_UTILS_PROTO_UTILS_H_
