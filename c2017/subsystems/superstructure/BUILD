load('//third_party/google/protobuf:protobuf.bzl', 'cc_proto_library')
package(default_visibility = ['//visibility:public'])

cc_proto_library(
  name = 'intake_group_proto',
  srcs = ['intake_group.proto'],
  protoc = '//third_party/google/protobuf:protoc',
  default_runtime = '//third_party/google/protobuf',
)

cc_proto_library(
  name = 'shooter_group_proto',
  srcs = ['shooter_group.proto'],
  protoc = '//third_party/google/protobuf:protoc',
  default_runtime = '//third_party/google/protobuf',
)

cc_library(
  name = 'intake_group_queues',
  hdrs = ['queue_types.h'],
  deps = [
    'intake_group_proto',
    '//muan/proto:stack_proto',
    '//muan/queues:queues'
  ],
)

cc_library(
  name = 'shooter_group_queues',
  hdrs = ['queue_types.h'],
  deps = [
    'shooter_group_proto',
    '//muan/proto:stack_proto',
    '//muan/queues:queues'
  ],
)

cc_library(
  name = 'superstructure_subsystem',
  hdrs = ['superstructure.h'],
  srcs = ['superstructure.cpp'],
  deps = [
      '//c2017/queue_manager:queue_manager',
  ],
  visibility = ['//visibility:public']
)
