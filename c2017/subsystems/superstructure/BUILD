load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "intake_group_proto",
    srcs = ["intake_group.proto"],
    default_runtime = "//third_party/google/protobuf",
    protoc = "//third_party/google/protobuf:protoc",
)

cc_proto_library(
    name = "shooter_group_proto",
    srcs = ["shooter_group.proto"],
    default_runtime = "//third_party/google/protobuf",
    protoc = "//third_party/google/protobuf:protoc",
)

cc_library(
    name = "intake_group_queues",
    hdrs = ["queue_types.h"],
    deps = [
        "intake_group_proto",
        "//muan/proto:stack_proto",
        "//muan/queues",
    ],
)

cc_library(
    name = "shooter_group_queues",
    hdrs = ["queue_types.h"],
    deps = [
        "shooter_group_proto",
        "//muan/proto:stack_proto",
        "//muan/queues",
    ],
)

cc_library(
    name = "superstructure_subsystem",
    srcs = ["superstructure.cpp"],
    hdrs = ["superstructure.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//c2017/queue_manager",
    ],
)
