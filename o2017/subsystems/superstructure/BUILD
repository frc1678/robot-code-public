load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "superstructure_proto",
    srcs = ["superstructure.proto"],
    default_runtime = "//third_party/google/protobuf",
    protoc = "//third_party/google/protobuf:protoc",
)

cc_library(
    name = "superstructure_queues",
    hdrs = ["queue_types.h"],
    deps = [
        "superstructure_proto",
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
        ":superstructure_queues",
        "//o2017/queue_manager",
        "//o2017/subsystems/superstructure/climber",
    ],
)
