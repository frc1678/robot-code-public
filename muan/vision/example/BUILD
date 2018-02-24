cc_binary(
    name = "vision_example",
    srcs = ["vision_example.cpp"],
    data = ["captured.avi"],
    linkopts = [
        "-lopencv_videoio",
        "-lopencv_highgui",
    ],
    deps = [
        "//muan/vision",
    ],
    restricted_to = ["//tools:k8", "//tools:armhf-debian"],
)
