load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

cc_proto_library(
    name = "commands_proto",
    srcs = ["commands.proto"],
    default_runtime = "//third_party/google/protobuf",
    protoc = "//third_party/google/protobuf:protoc",
)

cc_library(
    name = "command_queues",
    hdrs = ["queue_types.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":commands_proto",
        "//muan/proto:stack_proto",
        "//muan/queues:queue_manager",
    ],
)

cc_library(
    name = "commands",
    deps = [
        ":command_base",
        ":command_queues",
        ":drive_straight",
        ":test_auto",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "command_base",
    srcs = ["command_base.cpp"],
    hdrs = ["command_base.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":command_queues",
        "//muan/logging:logger",
        "//muan/queues:queue_manager",
        "//muan/subsystems/drivetrain:drivetrain_queues",
        "//muan/wpilib:wpilib_queues",
    ],
)

cc_test(
    name = "command_base_test",
    srcs = ["command_base_test.cpp"],
    linkopts = ["-lpthread"],
    deps = [
        ":command_base",
        ":test_auto",
        "//third_party/google/googletest",
    ],
)

cc_library(
    name = "test_auto",
    srcs = ["test_auto.cpp"],
    hdrs = ["test_auto.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":command_base",
    ],
)

cc_library(
    name = "drive_straight",
    srcs = ["drive_straight.cpp"],
    hdrs = ["drive_straight.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":command_base",
    ],
)
