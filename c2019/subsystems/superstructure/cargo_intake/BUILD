load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

cc_library(
    name = "cargo_intake",
    srcs = ["cargo_intake.cpp"],
    hdrs = ["cargo_intake.h"],
    visibility = ["//visibility:public"],
    deps = [
        "cargo_intake_proto",
        "cargo_intake_queues",
    ],
)
 
cc_library(
    name = "cargo_intake_queues",
    hdrs = ["queue_types.h"],
    visibility = ["//visibility:public"],
    deps = [
        "cargo_intake_proto",
        "//muan/proto:stack_proto",
    ],
)

cc_proto_library(
    name = "cargo_intake_proto",
    srcs = ["cargo_intake.proto"],
    default_runtime = "//third_party/google/protobuf",
    protoc = "//third_party/google/protobuf:protoc",
    visibility = ["//visibility:public"],
)
