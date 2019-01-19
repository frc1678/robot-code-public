load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

cc_library(
    name = "ground_hatch_intake",
    srcs = ["ground_hatch_intake.cpp"],
    hdrs = ["ground_hatch_intake.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":ground_hatch_intake_queues",
    ],
)

cc_library(
    name = "ground_hatch_intake_queues",
    hdrs = ["queue_types.h"],
    visibility = ["//visibility:public"],
    deps = [
        "ground_hatch_intake_proto",
        "//muan/proto:stack_proto",
    ],
)

cc_proto_library(
    name = "ground_hatch_intake_proto",
    srcs = ["ground_hatch_intake.proto"],
    default_runtime = "//third_party/google/protobuf",
    protoc = "//third_party/google/protobuf:protoc",
    visibility = ["//visibility:public"],
)

cc_test(
    name = "ground_hatch_intake_test",
    srcs = ["ground_hatch_intake_test.cpp"],
    deps = [
        ":ground_hatch_intake",
        "//third_party/google/googletest",
    ],
)
