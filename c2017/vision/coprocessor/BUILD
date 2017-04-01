cc_binary(
    name = "main",
    srcs = ["main.cpp"],
    deps = [
        ":sender",
        ":vision",
        "//third_party/gflags:gflags"
    ],
    linkopts = ["-lpthread"]
)

cc_library(
    name = "sender",
    srcs = ["sender.cpp"],
    hdrs = ["sender.h"],
    deps = [
        "//third_party/aos/common/util:phased_loop",
        "//third_party/aos/vision/events:udp",
        "//c2017/vision:vision_queues"
    ]
)

filegroup(
    name = "v4l_params_file",
    srcs = [
        "vision_camera_params",
        "stream_camera_params"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "vision",
    srcs = ["vision.cpp"],
    hdrs = ["vision.h"],
    deps = [
        ":sender",
        "//muan/vision:vision"
    ],
    data = ["robot_constants.pb.text", "thresholds.pb.text"],
    linkopts = [
        "-lopencv_video",
        "-lopencv_highgui",
    ],
    visibility = ["//visibility:public"],
)
