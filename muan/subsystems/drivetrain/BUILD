package(default_visibility = ["//visibility:public"])

load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

cc_proto_library(
    name = "drivetrain_proto",
    srcs = ["drivetrain.proto"],
    default_runtime = "//third_party/google/protobuf",
    protoc = "//third_party/google/protobuf:protoc",
)

cc_library(
    name = "drivetrain_queues",
    hdrs = ["queue_types.h"],
    deps = [
        "drivetrain_proto",
        "//muan/proto:stack_proto",
        "//muan/queues",
    ],
)

cc_library(
    name = "drivetrain_config",
    hdrs = [
        "drivetrain_config.h",
    ],
    deps = ["//muan/control:drivetrain_model"],
)

cc_library(
    name = "drivetrain",
    srcs = [
        "closed_loop_drive.cpp",
        "drivetrain.cpp",
        "open_loop_drive.cpp",
    ],
    hdrs = [
        "closed_loop_drive.h",
        "drivetrain.h",
        "open_loop_drive.h",
    ],
    linkopts = ["-lm"],
    deps = [
        ":drivetrain_config",
        ":drivetrain_queues",
        "//muan/control:nonlinear_feedback",
        "//muan/control:trajectory",
        "//muan/queues:queue_manager",
        "//muan/utils:math_utils",
        "//muan/wpilib:wpilib_queues",
    ],
)
