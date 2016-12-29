#ifndef MUAN_UTILS_PROTO_UTILS_HPP_
#define MUAN_UTILS_PROTO_UTILS_HPP_

#include "proto_utils.h"
#include "google/protobuf/generated_message_reflection.h"
#include "google/protobuf/message.h"
#include <iostream>
#include <sstream>
#include <vector>

namespace muan {

namespace util {

std::string FieldToCSV(
    const google::protobuf::Message& message,
    const google::protobuf::FieldDescriptor* const descriptor,
    const google::protobuf::Reflection* const reflection) {
  if (descriptor->is_repeated()) {
    aos::Die("Logging protos with repeated messages is not supported!");
  } else {
    switch (descriptor->cpp_type()) {
      case google::protobuf::FieldDescriptor::CPPTYPE_BOOL:
        return std::to_string(reflection->GetBool(message, descriptor));
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_ENUM:
        return reflection->GetEnum(message, descriptor)->name();
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_FLOAT:
        return std::to_string(reflection->GetFloat(message, descriptor));
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_INT32:
        return std::to_string(reflection->GetInt32(message, descriptor));
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_INT64:
        return std::to_string(reflection->GetInt64(message, descriptor));
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_DOUBLE:
        return std::to_string(reflection->GetDouble(message, descriptor));
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_STRING:
        return reflection->GetString(message, descriptor);
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_UINT32:
        return std::to_string(reflection->GetUInt32(message, descriptor));
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_UINT64:
        return std::to_string(reflection->GetUInt64(message, descriptor));
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE:
        auto& submessage = reflection->GetMessage(message, descriptor);
        return ProtoToCSV(submessage);
        break;
    }
  }
  return "";
}

std::string ProtoToCSV(const google::protobuf::Message& message) {
  auto reflection = message.GetReflection();
  std::vector<const google::protobuf::FieldDescriptor*> fields;
  reflection->ListFields(message, &fields);
  std::ostringstream ss;
  for (uint32_t i = 0; i < fields.size(); i++) {
    auto descriptor = fields[i];
    ss << FieldToCSV(message, descriptor, reflection);
    if (i != fields.size() - 1) {
      ss << ',';
    }
  }
  return ss.str();
}

std::string FieldToCSVHeader(
    const google::protobuf::Message& message,
    const google::protobuf::FieldDescriptor* const descriptor,
    const google::protobuf::Reflection* const reflection,
    const std::string& prefix) {
  if (descriptor->is_repeated()) {
    aos::Die("Logging protos with repeated messages is not supported!");
  } else {
    if (descriptor->cpp_type() ==
        google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE) {
      auto& submessage = reflection->GetMessage(message, descriptor);
      return ProtoToCSVHeader(submessage, prefix + descriptor->name() + ".");
    } else {
      return prefix + descriptor->name();
    }
  }
  return "";
}

std::string ProtoToCSVHeader(const google::protobuf::Message& message) {
  auto reflection = message.GetReflection();
  std::vector<const google::protobuf::FieldDescriptor*> fields;
  reflection->ListFields(message, &fields);
  std::ostringstream ss;
  for (uint32_t i = 0; i < fields.size(); i++) {
    auto descriptor = fields[i];
    ss << FieldToCSVHeader(message, descriptor, reflection, "");
    if (i != fields.size() - 1) {
      ss << ',';
    }
  }
  return ss.str();
}

std::string ProtoToCSVHeader(const google::protobuf::Message& message, const std::string& prefix) {
  auto reflection = message.GetReflection();
  std::vector<const google::protobuf::FieldDescriptor*> fields;
  reflection->ListFields(message, &fields);
  std::ostringstream ss;
  for (uint32_t i = 0; i < fields.size(); i++) {
    auto descriptor = fields[i];
    ss << FieldToCSVHeader(message, descriptor, reflection, prefix);
    if (i != fields.size() - 1) {
      ss << ',';
    }
  }
  return ss.str();
}

} /* util */

} /* muan */

#endif /* MUAN_UTILS_PROTO_UTILS_HPP_ */
