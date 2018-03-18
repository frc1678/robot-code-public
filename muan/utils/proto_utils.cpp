#ifndef MUAN_UTILS_PROTO_UTILS_HPP_
#define MUAN_UTILS_PROTO_UTILS_HPP_

#include "muan/utils/proto_utils.h"
#include <sstream>
#include <string>
#include <vector>
#include "google/protobuf/generated_message_reflection.h"
#include "google/protobuf/message.h"

namespace muan {
namespace util {

void DefaultFieldToCsv(const google::protobuf::FieldDescriptor* descriptor,
                       std::ostream& serialize);
void FieldToCsv(const google::protobuf::Message& message,
                const google::protobuf::Reflection* reflection,
                const google::protobuf::FieldDescriptor* descriptor,
                std::ostream& serialize);

void DefaultSubmessageToCsv(const google::protobuf::Descriptor* descriptor,
                            std::ostream& serialize) {
  for (int i = 0; i < descriptor->field_count(); i++) {
    DefaultFieldToCsv(descriptor->field(i), serialize);

    if (i != descriptor->field_count() - 1) {
      serialize << ',';
    }
  }
}

void SubmessageToCsv(const google::protobuf::Message& message,
                     const google::protobuf::Reflection* reflection,
                     const google::protobuf::Descriptor* descriptor,
                     std::ostream& serialize) {
  for (int i = 0; i < descriptor->field_count(); i++) {
    if (descriptor->field(i)->is_repeated()) {
      aos::Die("Logging protos with repeated messages is not supported!");
    } else if (reflection->HasField(message, descriptor->field(i))) {
      FieldToCsv(message, reflection, descriptor->field(i), serialize);
    } else {
      // The value is an unset optional, so we want a default value
      DefaultFieldToCsv(descriptor->field(i), serialize);
    }

    if (i != descriptor->field_count() - 1) {
      serialize << ',';
    }
  }
}

void FieldToCsv(const google::protobuf::Message& message,
                const google::protobuf::Reflection* reflection,
                const google::protobuf::FieldDescriptor* descriptor,
                std::ostream& serialize) {
  if (descriptor->is_repeated()) {
    aos::Die("Logging protos with repeated messages is not supported!");
  } else {
    switch (descriptor->cpp_type()) {
      case google::protobuf::FieldDescriptor::CPPTYPE_ENUM:
        serialize << reflection->GetEnumValue(message, descriptor);
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_STRING:
        serialize << reflection->GetString(message, descriptor);
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_FLOAT:
        serialize << reflection->GetFloat(message, descriptor);
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_DOUBLE:
        serialize << reflection->GetDouble(message, descriptor);
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_INT32:
        serialize << reflection->GetInt32(message, descriptor);
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_INT64:
        serialize << reflection->GetInt64(message, descriptor);
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_BOOL:
        serialize << reflection->GetBool(message, descriptor);
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_UINT32:
        serialize << reflection->GetUInt32(message, descriptor);
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_UINT64:
        serialize << reflection->GetUInt64(message, descriptor);
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE:
        const google::protobuf::Message& sub =
            reflection->GetMessage(message, descriptor);
        SubmessageToCsv(sub, sub.GetReflection(), descriptor->message_type(),
                        serialize);
        break;
    }
  }
}

void DefaultFieldToCsv(const google::protobuf::FieldDescriptor* descriptor,
                       std::ostream& serialize) {
  if (descriptor->is_repeated()) {
    aos::Die("Logging protos with repeated messages is not supported!");
  } else {
    switch (descriptor->cpp_type()) {
      case google::protobuf::FieldDescriptor::CPPTYPE_STRING:
        // Write an empty string by default
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_ENUM:
      case google::protobuf::FieldDescriptor::CPPTYPE_FLOAT:
      case google::protobuf::FieldDescriptor::CPPTYPE_DOUBLE:
      case google::protobuf::FieldDescriptor::CPPTYPE_INT32:
      case google::protobuf::FieldDescriptor::CPPTYPE_INT64:
      case google::protobuf::FieldDescriptor::CPPTYPE_BOOL:
      case google::protobuf::FieldDescriptor::CPPTYPE_UINT32:
      case google::protobuf::FieldDescriptor::CPPTYPE_UINT64:
        serialize << "0";
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE:
        DefaultSubmessageToCsv(descriptor->message_type(), serialize);
        break;
    }
  }
}

void ProtoToCsv(const google::protobuf::Message& message,
                std::ostream& serialize) {
  auto reflection = message.GetReflection();
  auto descriptor = message.GetDescriptor();

  SubmessageToCsv(message, reflection, descriptor, serialize);
}

void FieldToCsvHeader(const google::protobuf::FieldDescriptor* descriptor,
                      std::ostream& serialize, const char* prefix) {
  if (descriptor->is_repeated()) {
    aos::Die("Logging protos with repeated messages is not supported!");
  }
  if (descriptor->cpp_type() ==
      google::protobuf::FieldDescriptor::CppType::CPPTYPE_MESSAGE) {
    auto message = descriptor->message_type();
    for (int i = 0; i < message->field_count(); i++) {
      auto field = message->field(i);

      FieldToCsvHeader(
          field, serialize,
          (prefix + std::string(descriptor->name()) + ".").c_str());

      if (i != message->field_count() - 1) {
        serialize << ',';
      }
    }
  } else {
    serialize << prefix << descriptor->name();
  }
}

void ProtoToCsvHeader(const google::protobuf::Message& message,
                      std::ostream& serialize) {
  auto descriptor = message.GetDescriptor();

  if (descriptor->is_placeholder()) {
    return;
  }

  for (int i = 0; i < descriptor->field_count(); i++) {
    FieldToCsvHeader(descriptor->field(i), serialize, "");

    if (i != descriptor->field_count() - 1) {
      serialize << ',';
    }
  }
}

// JSON

void DefaultFieldToJson(const google::protobuf::FieldDescriptor* descriptor,
                        std::ostream& serialize);
void FieldToJson(const google::protobuf::Message& message,
                 const google::protobuf::Reflection* reflection,
                 const google::protobuf::FieldDescriptor* descriptor,
                 std::ostream& serialize);

void DefaultSubmessageToJson(const google::protobuf::Descriptor* descriptor,
                             std::ostream& serialize) {
  serialize << "{";
  for (int i = 0; i < descriptor->field_count(); i++) {
    DefaultFieldToJson(descriptor->field(i), serialize);

    if (i != descriptor->field_count() - 1) {
      serialize << ',';
    }
  }
  serialize << "}";
}

void SubmessageToJson(const google::protobuf::Message& message,
                      const google::protobuf::Reflection* reflection,
                      const google::protobuf::Descriptor* descriptor,
                      std::ostream& serialize) {
  serialize << "{";
  for (int i = 0; i < descriptor->field_count(); i++) {
    if (descriptor->field(i)->is_repeated()) {
      aos::Die("Logging protos with repeated messages is not supported!");
    } else if (reflection->HasField(message, descriptor->field(i))) {
      FieldToJson(message, reflection, descriptor->field(i), serialize);
    } else {
      // The value is an unset optional, so we want a default value
      DefaultFieldToJson(descriptor->field(i), serialize);
    }

    if (i != descriptor->field_count() - 1) {
      serialize << ',';
    }
  }
  serialize << "}";
}

void DefaultFieldToJson(const google::protobuf::FieldDescriptor* descriptor,
                        std::ostream& serialize) {
  if (descriptor->is_repeated()) {
    aos::Die("Logging protos with repeated messages is not supported!");
  } else {
    serialize << "\"" << descriptor->json_name() << "\":";
    switch (descriptor->cpp_type()) {
      case google::protobuf::FieldDescriptor::CPPTYPE_STRING:
        serialize << "\"\"";
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_ENUM:
      case google::protobuf::FieldDescriptor::CPPTYPE_FLOAT:
      case google::protobuf::FieldDescriptor::CPPTYPE_DOUBLE:
      case google::protobuf::FieldDescriptor::CPPTYPE_INT32:
      case google::protobuf::FieldDescriptor::CPPTYPE_INT64:
      case google::protobuf::FieldDescriptor::CPPTYPE_BOOL:
      case google::protobuf::FieldDescriptor::CPPTYPE_UINT32:
      case google::protobuf::FieldDescriptor::CPPTYPE_UINT64:
        serialize << "0";
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE:
        DefaultSubmessageToJson(descriptor->message_type(), serialize);
        break;
    }
  }
}

void FieldToJson(const google::protobuf::Message& message,
                 const google::protobuf::Reflection* reflection,
                 const google::protobuf::FieldDescriptor* descriptor,
                 std::ostream& serialize) {
  if (descriptor->is_repeated()) {
    aos::Die("Logging protos with repeated messages is not supported!");
  } else {
    serialize << "\"" << descriptor->json_name() << "\":";
    switch (descriptor->cpp_type()) {
      case google::protobuf::FieldDescriptor::CPPTYPE_ENUM:
        serialize << reflection->GetEnumValue(message, descriptor);
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_STRING:
        serialize << "\"" << reflection->GetString(message, descriptor) << "\"";
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_FLOAT:
        serialize << reflection->GetFloat(message, descriptor);
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_DOUBLE:
        serialize << reflection->GetDouble(message, descriptor);
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_INT32:
        serialize << reflection->GetInt32(message, descriptor);
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_INT64:
        serialize << reflection->GetInt64(message, descriptor);
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_BOOL:
        serialize << reflection->GetBool(message, descriptor);
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_UINT32:
        serialize << reflection->GetUInt32(message, descriptor);
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_UINT64:
        serialize << reflection->GetUInt64(message, descriptor);
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE:
        const google::protobuf::Message& sub =
            reflection->GetMessage(message, descriptor);
        SubmessageToJson(sub, sub.GetReflection(), descriptor->message_type(),
                         serialize);
        break;
    }
  }
}

void ProtoToJson(const google::protobuf::Message& message,
                 std::ostream& serialize) {
  auto reflection = message.GetReflection();
  auto descriptor = message.GetDescriptor();

  SubmessageToJson(message, reflection, descriptor, serialize);
}

}  // namespace util
}  // namespace muan

#endif  // MUAN_UTILS_PROTO_UTILS_HPP_
