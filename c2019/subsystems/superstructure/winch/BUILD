load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "winch_proto",
    srcs = ["winch.proto"],
    default_runtime = "//third_party/google/protobuf",
    protoc = "//third_party/google/protobuf:protoc",
)

cc_library(
    name = "winch_queues",
    hdrs = ["queue_types.h"],
    deps = [
        ":winch_proto",
        "//muan/proto:stack_proto",
        "//muan/queues",
    ],
)


cc_library(
    name = "winch",
    srcs = ["winch.cpp"],
    hdrs = ["winch.h"],
    deps = [
        "//muan/wpilib:wpilib_queues",
        "//muan/queues:queue_manager",
        ":winch_queues",
    ],
)

cc_test(
    name = "winch_test",
    srcs = ["winch_test.cpp"],
    deps = [
        ":winch",
        "//third_party/google/googletest",
    ],
)
