load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "score_subsystem_proto",
    srcs = ["score_subsystem.proto"],
    default_runtime = "//third_party/google/protobuf",
    protoc = "//third_party/google/protobuf:protoc",
)

cc_library(
    name = "score_subsystem_queues",
    hdrs = ["queue_types.h"],
    restricted_to = ["//tools:roborio"],
    deps = [
    	"score_subsystem_proto",
        "//muan/proto:stack_proto",
        "//muan/queues",
    ],
)
