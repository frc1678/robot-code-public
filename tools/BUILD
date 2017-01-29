load("//third_party/google/protobuf:protobuf.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

exports_files(["test_sharding_compliant"])

action_listener(
    name = "compilation_database",
    extra_actions = [
        ":cpp_compilation_commands_gatherer",
    ],
    mnemonics = [
        "CppCompile",
    ],
    visibility = ["//visibility:public"],
)

extra_action(
    name = "cpp_compilation_commands_gatherer",
    cmd = "$(location :gather_compilation_commands) $(EXTRA_ACTION_FILE) $(output out_$(ACTION_ID).json)",
    out_templates = ["out_$(ACTION_ID).json"],
    tools = [":gather_compilation_commands"],
)

py_binary(
    name = "gather_compilation_commands",
    srcs = ["gather_cdb.py"],
    main = "gather_cdb.py",
    deps = ["//third_party/google/bazel:compile_info"],
)
