load("/third_party/google/protobuf/protobuf", "cc_proto_library")

cc_proto_library(
    name = "magazine_proto",
    srcs = ["magazine.proto"],
    default_runtime = "//third_party/google/protobuf",
    protoc = "//third_party/google/protobuf:protoc",
)

cc_library(
    name = "magazine_queues",
    hdrs = ["queue_types.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":magazine_proto",
        "//muan/proto:stack_proto",
        "//muan/queues",
    ],
)
