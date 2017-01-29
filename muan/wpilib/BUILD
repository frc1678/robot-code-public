package(default_visibility = ["//visibility:public"])

load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

cc_library(
    name = "can_wrapper",
    srcs = ["can_wrapper.cpp"],
    hdrs = ["can_wrapper.h"],
    deps = [
        ":pcm_wrapper",
        ":pdp_wrapper",
        "//third_party/aos/common:time",
        "//third_party/aos/common/util:phased_loop",
        "//third_party/aos/linux_code:init",
    ],
)

cc_library(
    name = "pdp_wrapper",
    srcs = ["pdp_wrapper.cpp"],
    hdrs = ["pdp_wrapper.h"],
    deps = [
        ":state_protos",
        "//muan/proto:stack_proto",
        "//muan/queues",
        "//third_party/aos/common:die",
        "//third_party/wpilibsuite:wpilib",
    ],
)

cc_library(
    name = "pcm_wrapper",
    srcs = ["pcm_wrapper.cpp"],
    hdrs = ["pcm_wrapper.h"],
    deps = [
        "//third_party/aos/common:die",
        "//third_party/wpilibsuite:wpilib",
    ],
)

cc_binary(
    name = "pdp_test",
    srcs = ["pdp_test.cpp"],
    deps = [
        ":can_wrapper",
        "//third_party/wpilibsuite:wpilib",
    ],
)

cc_binary(
    name = "pcm_test",
    srcs = ["pcm_test.cpp"],
    deps = [
        ":can_wrapper",
        "//third_party/wpilibsuite:wpilib",
    ],
)

cc_library(
    name = "wpilib_queues",
    hdrs = ["queue_types.h"],
    deps = [
        ":state_protos",
        "//muan/proto:stack_proto",
        "//muan/queues",
    ],
)

cc_proto_library(
    name = "state_protos",
    srcs = ["state.proto"],
    default_runtime = "//third_party/google/protobuf",
    protoc = "//third_party/google/protobuf:protoc",
)
