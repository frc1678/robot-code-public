load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

cc_library(
    name = "limelight_queues",
    hdrs = ["queue_types.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":limelight_proto",
        "//muan/proto:stack_proto",
        "//muan/queues:queue_manager",
    ],
)

cc_proto_library(
    name = "limelight_proto",
    srcs = ["limelight.proto"],
    default_runtime = "//third_party/google/protobuf",
    protoc = "//third_party/google/protobuf:protoc",
)

cc_library(
    name = "limelight",
    srcs = ["limelight.cpp"],
    hdrs = ["limelight.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":limelight_queues",
        "//c2019/subsystems/superstructure:superstructure_queues",
        "//muan/queues:queue_manager",
        "//muan/wpilib:wpilib_queues",
        "//third_party/wpilibsuite:ntcore",
    ],
)
