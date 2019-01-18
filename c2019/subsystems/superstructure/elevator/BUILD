package(default_visibility = ["//visibility:public"])

load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

cc_library(
   name = "elevator",
   srcs = ["elevator.cpp"],
   hdrs = ["elevator.h"],
   deps = [
        "elevator_proto",
        "elevator_queues",
        "//muan/logging:logger",
        "//muan/utils:math_utils",
        "//muan/utils:disk_brake",
    ],
)

cc_library(
    name = "elevator_queues",
    hdrs = ["queue_types.h"],
    deps = [
            "elevator_proto",
            "//muan/proto:stack_proto",
            "//muan/queues",
    ],
)

cc_proto_library(
    name = "elevator_proto",
    srcs = ["elevator.proto"],
    default_runtime = "//third_party/google/protobuf",
    protoc = "//third_party/google/protobuf:protoc",
)

cc_test(
    name = "elevator_test",
    srcs = ["elevator_test.cpp"],
    deps = [
            ":elevator",
            "//third_party/google/googletest",
    ],
)
