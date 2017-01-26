load("/third_party/google/protobuf/protobuf", "cc_proto_library")

cc_library(
    name = "queue_types",
    hdrs = ["queue_types.h"],
    visibility = ["//visibility:public"],
    deps = [
        "drivetrain_protos",
        "//muan/proto:stack_proto",
        "//muan/queues",
    ],
)

cc_proto_library(
    name = "drivetrain_protos",
    srcs = ["drivetrain.proto"],
    default_runtime = "//third_party/google/protobuf",
    protoc = "//third_party/google/protobuf:protoc",
)
