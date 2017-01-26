load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

cc_library(
    name = "catapult_queues",
    hdrs = ["queue_types.h"],
    visibility = ["//visibility:public"],
    deps = [
        "catapult_proto",
        "//muan/proto:stack_proto",
        "//muan/queues",
    ],
)

cc_proto_library(
    name = "catapult_proto",
    srcs = ["catapult.proto"],
    default_runtime = "//third_party/google/protobuf",
    protoc = "//third_party/google/protobuf:protoc",
    visibility = ["//visibility:public"],
)

cc_test(
    name = "catapult_queues_test",
    srcs = ["queue_types_test.cpp"],
    deps = [
        ":catapult_queues",
        "//third_party/google/googletest",
    ],
)
