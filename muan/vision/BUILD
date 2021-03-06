load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

cc_library(
    name = "vision",
    srcs = ["vision.cpp"],
    hdrs = ["vision.h"],
    linkopts = [
        "-lopencv_imgproc",
        "-lopencv_core",
        "-lopencv_videoio",
        "-lopencv_highgui",
        "-lopencv_imgcodecs",
    ],
    data = ["//c2017/vision/coprocessor:v4l_params_file"],
    deps = [
        "@opencv//:opencv",
        ":config_proto",
        "//muan/logging:logger",
    ],
    visibility = ["//visibility:public"],
    restricted_to = ["//tools:k8", "//tools:armhf-debian"],
)

cc_binary(
    name = 'calibration_helper',
    srcs = ['calibration_helper.cpp'],
    linkopts = [
        "-lopencv_imgproc",
        "-lopencv_core",
        "-lopencv_videoio",
        "-lopencv_highgui",
    ],
    data = ["//c2017/vision/coprocessor:v4l_params_file"],
    deps = [
        "//third_party/gflags",
        ":config_proto",
        ":vision",
    ],
    restricted_to = ["//tools:k8", "//tools:armhf-debian"],
)

cc_binary(
    name = "test_video_stream",
    srcs = ["test_video_stream.cpp"],
    deps = [
      "//muan/webdash:server",
      ":video_stream",
    ],
    linkopts = [
      "-lopencv_imgproc",
      "-lopencv_videoio",
    ],
    restricted_to = ["//tools:k8", "//tools:armhf-debian"],
)

cc_library(
    name = "video_stream",
    srcs = ["video_stream.cpp"],
    hdrs = ["video_stream.h"],
    deps = [
        "//muan/webdash:server",
        "//muan/webdash:webdash_queues",
        "//third_party/optional:optional",
        "//third_party/aos/common:die",
        "//third_party/aos/common/util:phased_loop",
        "//muan/logging:textlogger",
        "//muan/queues:queues",
        "//muan/logging:logger",
    ],
    linkopts = [
        "-lopencv_core",
        "-lopencv_highgui",
        "-lopencv_imgcodecs",
    ],
    restricted_to = ["//tools:k8", "//tools:armhf-debian"],
)

cc_proto_library(
    name = "config_proto",
    srcs = ["config.proto"],
    protoc = "//third_party/google/protobuf:protoc",
    default_runtime = "//third_party/google/protobuf",
    visibility = ["//visibility:public"],
)

cc_test(
    name = "vision_test",
    srcs = ["vision_test.cpp"],
    deps = [
        ":vision",
        "//third_party/google/googletest",
    ],
    linkopts = ["-lopencv_highgui"],
    restricted_to = ["//tools:k8", "//tools:armhf-debian"],
)
