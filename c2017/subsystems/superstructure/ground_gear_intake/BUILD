load('//third_party/google/protobuf:protobuf.bzl', 'cc_proto_library')

cc_library(
  name = 'ground_gear_intake_queues',
  hdrs = ['queue_types.h'],
  deps = [
    'ground_gear_intake_proto',
    '//muan/proto:stack_proto',
    '//muan/queues:queues'
  ],
  visibility = ['//visibility:public']
)

cc_proto_library(
  name = 'ground_gear_intake_proto',
  srcs = ['ground_gear_intake.proto'],
  protoc = '//third_party/google/protobuf:protoc',
  default_runtime = '//third_party/google/protobuf',
  visibility = ['//visibility:public']
)
