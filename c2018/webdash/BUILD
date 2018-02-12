cc_library(
    name = "webdash_c2018",
    srcs = ["webdash_setup.cpp"],
    hdrs = ["webdash_setup.h"],
    deps = [
        "//muan/webdash:server",
        "//muan/queues:queues",
        "//muan/queues:queue_manager",
    ],
    data = [
        "//c2018/webdash:json",
    ],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "json",
    srcs = ["display_object.json"],
    visibility = ["//visibility:public"],
)
