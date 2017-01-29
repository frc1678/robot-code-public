load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")
package(default_visibility = ["//visibility:public"])

cc_binary(
  name = "test_server",
  srcs = ["test_server.cpp"],
  deps = [":server"]
)

filegroup(
    name = "webdash_html",
    srcs = glob(["www/**"]),
)

cc_library(
  name = "server",
  srcs = ["server.cpp"],
  hdrs = ["server.h"],
  deps = [
    "//third_party/mongoose_cpp:mongoose_cpp",
    "//c2017/queue_manager:queue_manager"],
  linkopts = ["-lpthread"]
)

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
