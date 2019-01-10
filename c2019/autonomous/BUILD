load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

cc_proto_library(
    name = "autonomous_proto",
    srcs = ["autonomous.proto"],
    default_runtime = "//third_party/google/protobuf",
    protoc = "//third_party/google/protobuf:protoc",
)

cc_library(
    name = "autonomous_queues",
    hdrs = ["queue_types.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":autonomous_proto",
        "//muan/proto:stack_proto",
        "//muan/queues:queue_manager",
    ],
)

cc_library(
    name = "autonomous_runner",
    srcs = ["autonomous_runner.cpp"],
    hdrs = ["autonomous_runner.h"],
    # restricted_to = ["//tools:roborio"],
    visibility = ["//visibility:public"],
    deps = [
        ":autonomous_base",
        ":autonomous_queues",
        ":test_auto",
        ":none",
        ":drive_straight",
        "//muan/logging:logger",
        "//muan/queues:queue_manager",
    ],
)

cc_library(
    name = "autonomous_base",
    srcs = ["autonomous_base.cpp"],
    hdrs = ["autonomous_base.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":autonomous_queues",
        "//muan/logging:logger",
        "//muan/queues:queue_manager",
        "//muan/subsystems/drivetrain:drivetrain_queues",
        "//muan/wpilib:wpilib_queues",
    ],
)

cc_test(
    name = "autonomous_base_test",
    srcs = ["autonomous_base_test.cpp"],
    deps = [
        ":autonomous_base",
        "//third_party/google/googletest",
    ],
)

cc_library(
    name = "test_auto",
    srcs = ["test_auto.cpp"],
    hdrs = ["test_auto.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":autonomous_base",
    ],
)

cc_library(
    name = "none",
    srcs = ["none.cpp"],
    hdrs = ["none.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":autonomous_base",
    ],
)

cc_library(
    name = "drive_straight",
    srcs = ["drive_straight.cpp"],
    hdrs = ["drive_straight.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":autonomous_base",
    ],
)
