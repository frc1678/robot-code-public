package(default_visibility = ["//visibility:public"])
load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

cc_proto_library(
    name = "arm_proto",
    srcs = ["arm.proto"],
    default_runtime = "//third_party/google/protobuf",
    protoc = "//third_party/google/protobuf:protoc",
)

cc_library(
    name = "arm_queues",
    hdrs = ["queue_types.h"],
    deps = [
        "arm_proto",
        "//muan/proto:stack_proto",
        "//muan/queues",
    ],
)

cc_library(
    name = "arm",
    srcs = ["arm.cpp"],
    hdrs = ["arm.h"],
    deps = [
        "arm_queues",
        "//muan/wpilib:wpilib_queues",
        "//muan/queues:queue_manager",
        "//muan/logging:logger",
        "//muan/units",
        "//muan/utils:math_utils",
    ],
)

cc_test(
    name = "arm_test",
    srcs = ["arm_test.cpp"],
    deps = [
        ":arm",
        "//third_party/google/googletest",
    ],
)
