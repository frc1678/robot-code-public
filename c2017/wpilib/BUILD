load('//third_party/google/protobuf:protobuf.bzl', 'cc_proto_library')

cc_library(
  name = 'wpilib_queues',
  hdrs = [
    'queue_types.h',
  ],
  deps = [
    'wpilib_proto',
    '//muan/proto:stack_proto',
    '//muan/queues:queues'
  ],
  visibility = ['//visibility:public']
)

cc_proto_library(
  name = 'wpilib_proto',
  srcs = ['wpilib.proto'],
  protoc = '//third_party/google/protobuf:protoc',
  default_runtime = '//third_party/google/protobuf',
  visibility = ['//visibility:public']
)

cc_library(
  name = 'wpilib_interface',
  srcs = [
    'wpilib_interface.cpp',
  ],
  hdrs = [
    'wpilib_interface.h',
  ],
  deps = [
    '//c2017/queue_manager',
    '//muan/wpilib:can_wrapper',
    '//muan/wpilib/gyro:gyro',
    '//third_party/wpilibsuite:wpilib',
  ],
  visibility = ['//visibility:public']
)

