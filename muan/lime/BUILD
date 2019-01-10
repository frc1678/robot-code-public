load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

cc_library(
    name = "lime_queues",
    hdrs = ["queue_types.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":lime_proto",
        "//muan/proto:stack_proto",
        "//muan/queues:queue_manager",
    ],
)

cc_proto_library(
    name = "lime_proto",
    srcs = ["lime.proto"],
    default_runtime = "//third_party/google/protobuf",
    protoc = "//third_party/google/protobuf:protoc",
)

cc_library(
    name = "lime",
    srcs = ["lime.cpp"],
    hdrs = ["lime.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":lime_queues",
        "//muan/queues:queue_manager",
        "//muan/wpilib:wpilib_queues",
        "//third_party/wpilibsuite:ntcore",
    ],
)

cc_test(
    name = "lime_test",
    srcs = ["lime_test.cpp"],
    deps = [
        ":lime",
        "//third_party/google/googletest",
    ],
)
