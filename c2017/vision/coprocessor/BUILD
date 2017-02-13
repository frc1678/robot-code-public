cc_binary(
    name = "main",
    srcs = ["main.cpp"],
    deps = [":sender", ":vision"],
    data = glob(["*.avi"]), # For debugging
    linkopts = ["-lpthread"]
)

cc_library(
    name = "sender",
    srcs = ["sender.cpp"],
    hdrs = ["sender.h"],
    deps = [
        "//third_party/aos/vision/events:udp",
        "//c2017/vision:vision_queues"
    ]
)

cc_library(
    name = "vision",
    srcs = ["vision.cpp"],
    hdrs = ["vision.h"],
    deps = [
        ":sender",
        "//muan/vision:vision"
    ],
    linkopts = ["-lopencv_video", "-lopencv_highgui"]
)
