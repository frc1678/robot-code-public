#ifndef MUAN_UTILS_PROTO_UTILS_H_
#define MUAN_UTILS_PROTO_UTILS_H_

#include "google/protobuf/message.h"
#include <string>

namespace muan {

namespace util {

std::string ProtoToCSV(const google::protobuf::Message& message);

std::string ProtoToCSVHeader(const google::protobuf::Message& message);

} /* util */

} /* muan */

#endif /* MUAN_UTILS_PROTO_UTILS_H_ */
