load('//third_party/google/protobuf:protobuf.bzl', 'cc_proto_library')

cc_library(
  name = 'intake_queues',
  hdrs = ['queue_types.h'],
  deps = ['intake_proto', '//muan/proto:stack_proto', '//muan/queues:queues']
)

cc_proto_library(
  name = 'intake_proto',
  srcs = ['intake.proto'],
  protoc = '//third_party/google/protobuf:protoc',
  default_runtime = '//third_party/google/protobuf'
)

