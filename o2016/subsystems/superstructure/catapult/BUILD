load('//third_party/google/protobuf:protobuf.bzl', 'cc_proto_library')

cc_library(
  name = 'catapult_queues',
  hdrs = ['queue_types.h'],
  deps = ['catapult_proto', '//muan/proto:stack_proto', '//muan/queues:queues'],
  visibility = ['//visibility:public']
)

cc_proto_library(
  name = 'catapult_proto',
  srcs = ['catapult.proto'],
  protoc = '//third_party/google/protobuf:protoc',
  default_runtime = '//third_party/google/protobuf',
  visibility = ['//visibility:public']
)

cc_test(
  name = 'catapult_queues_test',
  srcs = ['queue_types_test.cpp'],
  deps = [
     '//third_party/google/googletest:googletest',
     ':catapult_queues'
  ]
)
