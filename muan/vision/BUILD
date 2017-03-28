load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

cc_library(
    name = "vision",
    srcs = ["vision.cpp"],
    hdrs = ["vision.h"],
    linkopts = [
        "-lopencv_imgproc",
        "-lopencv_core",
    ],
    deps = [":config_proto"],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = 'calibration_helper',
    srcs = ['calibration_helper.cpp'],
    linkopts = [
        "-lopencv_imgproc",
        "-lopencv_core",
        "-lopencv_video",
        "-lopencv_highgui",
    ],
    data = ["//c2017/vision/coprocessor:v4l_params_file"],
    deps = [
        "//third_party/gflags",
        ":config_proto",
        ":vision",
    ],
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
    linkopts = ["-lopencv_highgui"]
)
