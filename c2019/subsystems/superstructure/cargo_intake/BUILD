load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

cc_library(
    name = "cargo_intake_queues",
    hdrs = ["queue_types.h"],
    visibility = ["//visibility:public"],
    deps = [
        "cargo_intake_proto",
        "//muan/proto:stack_proto",
        "//muan/queues",
    ],
)

cc_proto_library(
    name = "cargo_intake_proto",
    srcs = ["cargo_intake.proto"],
    default_runtime = "//third_party/google/protobuf",
    protoc = "//third_party/google/protobuf:protoc",
    visibility = ["//visibility:public"],
)
