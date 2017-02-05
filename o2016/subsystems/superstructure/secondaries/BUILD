load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "secondaries",
    srcs = ["secondaries.cpp"],
    hdrs = ["secondaries.h"],
    deps = [
        ":secondaries_proto",
        ":secondaries_queues",
        "//muan/proto:stack_proto",
    ],
)

cc_library(
    name = "secondaries_queues",
    hdrs = ["queue_types.h"],
    deps = [
        "secondaries_proto",
        "//muan/proto:stack_proto",
        "//muan/queues",
    ],
)

cc_proto_library(
    name = "secondaries_proto",
    srcs = ["secondaries.proto"],
    default_runtime = "//third_party/google/protobuf",
    protoc = "//third_party/google/protobuf:protoc",
)

cc_test(
    name = "secondaries_test",
    srcs = ["test_secondaries.cpp"],
    deps = [
        "secondaries",
        "//third_party/google/googletest",
    ],
)
