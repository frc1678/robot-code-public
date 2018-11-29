cc_library(
    name = "webdash",
    srcs = ["webdash_setup.cpp"],
    hdrs = ["webdash_setup.h"],
    deps = [
        "//muan/webdash:server",
        "//muan/queues:queues",
        "//muan/queues:queue_manager",
    ],
    data = [
        "//o2018/webdash:json",
    ],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "json",
    srcs = ["display_object.json"],
    visibility = ["//visibility:public"],
)
