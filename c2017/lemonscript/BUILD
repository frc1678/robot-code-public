filegroup(
    name = "lemonscript_funcs",
    srcs = glob(["funcs/*.func"]),
)

genrule(
    name = "lemonscript_generate",
    srcs = [
        ":lemonscript_funcs",
    ],
    outs = [
        "ls_gen.cpp",
        "ls_gen.h",
    ],
    cmd = "$(location //third_party/lemonscript_transpiler:lemonscript_transpiler) --input-files $(locations :lemonscript_funcs) --output-cpp $(location ls_gen.cpp) --output-header $(location ls_gen.h) -qq",
    tools = [
        "//third_party/lemonscript_transpiler",
    ],
)

cc_library(
    name = "lemonscript_funcs_cpp",
    srcs = [
        "ls_gen.cpp",
    ],
    hdrs = [
        "ls_gen.h",
    ],
    copts = ["-Wno-unused-variable"],
    deps = [
        "//c2017/queue_manager",
        "//c2017/subsystems/drivetrain:drivetrain_base",
        "//c2017/subsystems/superstructure:superstructure_subsystem",
        "//muan/actions:drivetrain_action",
        "//third_party/lemonscript",
    ],
)

cc_library(
    name = "lemonscript",
    srcs = [
        "lemonscript.cpp",
        "lemonscript.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":lemonscript_funcs_cpp",
        "//c2017/queue_manager",
        "//c2017/subsystems/drivetrain:drivetrain_base",
        "//c2017/subsystems/superstructure:superstructure_subsystem",
        "//muan/utils:threading_utils",
        "//muan/actions:drivetrain_action",
        "//third_party/aos/common:time",
        "//third_party/aos/common/util:phased_loop",
        "//third_party/aos/linux_code:init",
        "//third_party/lemonscript",
        "//third_party/gflags:gflags"
    ],
)

cc_binary(
    name = "test",
    srcs = ["test.cpp"],
    deps = [":lemonscript"],
    data = ["//c2017/lemonscript/auto:auto_modes"],
)
