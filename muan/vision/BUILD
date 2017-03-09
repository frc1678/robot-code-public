cc_library(
    name = "vision",
    srcs = ["vision.cpp"],
    hdrs = ["vision.h"],
    linkopts = [
        "-lopencv_imgproc",
        "-lopencv_core",
    ],
    visibility = ["//visibility:public"],
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
        ":vision",
        "//c2017/vision:vision_proto",
        "//c2017/vision/coprocessor:vision",
    ],
)
