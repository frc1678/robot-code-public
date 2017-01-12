load('//third_party/google/protobuf:protobuf.bzl', 'cc_proto_ibrary')

cc_library(
  name = 'climber_queues',
  hdrs = ['queue_types.h'],
  deps = ['climber_proto', '//muan/proto:stack_proto', '//muan/queues:queues'],
  visibility = ['//visibility:public']
)

cc_proto_library(
  name = 'climber_proto',
  srcs = ['climber.proto'],
  protoc = '//third_party/google/protobuf',
  visibility = ['//visibility:public']
)
