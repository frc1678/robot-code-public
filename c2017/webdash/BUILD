load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

cc_proto_library(
  name = "webdash_proto",
  srcs = ["webdash.proto"],
  default_runtime = "//third_party/google/protobuf",
  protoc = "//third_party/google/protobuf:protoc",
)

cc_library(
  name = 'webdash_queues',
  hdrs = ['queue_types.h'],
  deps = [
    'webdash_proto',
    '//muan/queues',
    '//muan/proto:stack_proto'
  ]
)
