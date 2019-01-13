load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

cc_library(
    name = "wrist_queues",
    hdrs = ["queue_types.h"],
    visibility = ["//visibility:public"],
    deps = [
        "wrist_proto",
        "//muan/proto:stack_proto",
        "//muan/queues",
    ],
)

cc_proto_library(
    name = "wrist_proto",
    srcs = ["wrist.proto"],
    default_runtime = "//third_party/google/protobuf",
    protoc = "//third_party/google/protobuf:protoc",
    visibility = ["//visibility:public"],
)
