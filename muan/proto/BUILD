load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

cc_library(
    name = "stack_proto",
    srcs = ["stack_proto.cpp"],
    hdrs = ["stack_proto.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//third_party/aos/common:die",
        "//third_party/aos/common:time",
        "//third_party/google/protobuf",
    ],
)

cc_proto_library(
    name = "test_proto",
    srcs = ["test_proto.proto"],
    default_runtime = "//third_party/google/protobuf",
    protoc = "//third_party/google/protobuf:protoc",
)

cc_test(
    name = "stack_proto_test",
    srcs = ["stack_proto_test.cpp"],
    deps = [
        ":stack_proto",
        ":test_proto",
        "//muan/queues",
        "//third_party/google/googletest",
    ],
)
