load('//third_party/google/protobuf:protobuf.bzl', 'cc_proto_library')

cc_library(
  name = 'gyro_queues',
  hdrs = ['queue_types.h'],
  deps = [':gyro_protos'],
  visibility = ['//visibility:public'],
)

cc_library(
  name = 'gyro',
  srcs = ['gyro_interface.cpp', 'gyro_reader.cpp'],
  hdrs = ['gyro_interface.h', 'gyro_reader.h'],
  deps = [
    '//third_party/wpilibsuite:wpilib',
    '//muan/units',
    '//third_party/aos/common/util:phased_loop',
    ':gyro_queues',
    '//muan/proto:stack_proto',
    '//muan/queues',
  ],
  visibility = ['//visibility:public'],
)

cc_proto_library(
  name = 'gyro_protos',
  srcs = ['gyro.proto'],
  protoc = '//third_party/google/protobuf:protoc',
  default_runtime = '//third_party/google/protobuf'
)
