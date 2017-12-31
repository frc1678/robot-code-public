package(default_visibility = ["//visibility:public"])

load("//tools/cpp:toolchain_macros.bzl", "generate_cc_toolchains", "generate_cc_toolchain_suite")

cc_library(
    name = "malloc",
)

cc_library(
    name = "stl",
)

filegroup(
    name = "empty",
    srcs = [],
)

# This is the entry point for --crosstool_top.  Toolchains are found
# by lopping off the name of --crosstool_top and searching for
# "cc-compiler-${CPU}" in this BUILD file, where CPU is the target CPU
# specified in --cpu.

# All of the cpus we want to support
toolchain_cpus = [
    "armeabi-v7a",
    "darwin",
    "freebsd",
    "local",
    "k8",
    "roborio",
    "roborio-osxbuild",
    "empty",
]

generate_cc_toolchain_suite("toolchain", toolchain_cpus)

generate_cc_toolchains(toolchain_cpus)

filegroup(
    name = "srcs",
    srcs = glob(["**"]),
)
