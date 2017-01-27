package(default_visibility = ["//visibility:public"])

cc_library(
    name = "queues",
    srcs = ["message_queue.hpp"],
    hdrs = ["message_queue.h"],
    linkstatic = 1,
    deps = [
        "//muan/utils:math_utils",
        "//third_party/aos/common:mutex",
        "//third_party/optional",
    ],
)

cc_test(
    name = "queues_test",
    srcs = ["message_queue_test.cpp"],
    deps = [
        ":queues",
        "//third_party/google/googletest",
    ],
)
