load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

cc_library(
    name = "ground_ball_intake",
    srcs = ["ground_ball_intake.cpp"],
    hdrs = ["ground_ball_intake.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":ground_ball_intake_queues",
        "//c2017/queue_manager",
        "//muan/wpilib:wpilib_queues",
    ],
)

cc_library(
    name = "ground_ball_intake_queues",
    hdrs = ["queue_types.h"],
    visibility = ["//visibility:public"],
    deps = [
        "ground_ball_intake_proto",
        "//muan/proto:stack_proto",
        "//muan/queues",
    ],
)

cc_proto_library(
    name = "ground_ball_intake_proto",
    srcs = ["ground_ball_intake.proto"],
    default_runtime = "//third_party/google/protobuf",
    protoc = "//third_party/google/protobuf:protoc",
    visibility = ["//visibility:public"],
)

cc_test(
    name = "ground_ball_intake_test",
    srcs = ["ground_ball_intake_test.cpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":ground_ball_intake",
        ":ground_ball_intake_queues",
        "//muan/wpilib:wpilib_queues",
        "//third_party/google/googletest",
    ],
)
