load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

cc_library(
    name = "lights_queues",
    hdrs = ["queue_types.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":lights_proto",
        "//muan/proto:stack_proto",
        "//muan/queues:queue_manager",
    ],
)

cc_proto_library(
    name = "lights_proto",
    srcs = ["lights.proto"],
    default_runtime = "//third_party/google/protobuf",
    protoc = "//third_party/google/protobuf:protoc",
)

cc_library(
    name = "lights",
    srcs = ["lights.cpp"],
    hdrs = ["lights.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":lights_queues",
        "//muan/queues:queue_manager",
        "//muan/wpilib:wpilib_queues",
        "//c2018/subsystems/score_subsystem:score_subsystem_queues",
    ],
)

cc_test(
    name = "lights_test",
    srcs = ["lights_test.cpp"],
    deps = [
        ":lights",
        "//third_party/google/googletest",
    ],
)

