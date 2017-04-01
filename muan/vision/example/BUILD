cc_binary(
    name = "vision_example",
    srcs = ["vision_example.cpp"],
    data = ["captured.avi"],
    linkopts = [
        "-lopencv_video",
        "-lopencv_highgui",
    ],
    deps = [
        "//muan/vision",
    ],
)
