cc_library(
    name = "lemonscript",
    srcs = [
        "lemonscript.cpp",
        "lemonscript.h",
    ],
    deps = [
        "//third_party/aos/common:time",
        "//third_party/aos/common/util:phased_loop",
        "//third_party/aos/linux_code:init",
        "//third_party/lemonscript",
        "//third_party/lemonscript_transpiler:example_func",
    ],
)

cc_binary(
    name = "test",
    srcs = ["test.cpp"],
    deps = [":lemonscript"],
)
