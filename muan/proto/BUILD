load('//third_party/google/protobuf:protobuf.bzl', 'cc_proto_library')

cc_library(
  name = 'stack_proto',
  srcs = ['stack_proto.cpp'],
  hdrs = ['stack_proto.h'],
  deps = [
    '//third_party/google/protobuf',
    '//third_party/aos/common:die'
  ]
)

cc_proto_library(
  name = 'test_proto',
  srcs = ['test_proto.proto'],
  protoc = '//third_party/google/protobuf:protoc',
  default_runtime = '//third_party/google/protobuf'
)

cc_test(
  name = 'stack_proto_test',
  srcs = ['stack_proto_test.cpp'],
  deps = [
    ':stack_proto',
    ':test_proto',
    '//third_party/google/googletest',
    '//muan/queues',
  ],
)
