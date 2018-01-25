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

config_setting(
  name = 'compiler_clang',
  values = {'compiler': 'clang'}
)
config_setting(
  name = 'compiler_gcc',
  values = {'compiler': 'gcc'}
)
config_setting(
  name = 'cpu_k8',
  values = {'cpu': 'k8'},
)
config_setting(
  name = 'cpu_roborio',
  values = {'cpu': 'roborio'},
)
config_setting(
  name = 'cpu_cortex_m4f',
  values = {'cpu': 'cortex-m4f'},
)
config_setting(
  name = 'cpu_armhf',
  values = {'cpu': 'armhf-debian'},
)
config_setting(
  name = 'has_asan',
  values = {'define': 'have_asan=true'},
)
config_setting(
  name = 'has_tsan',
  values = {'define': 'have_tsan=true'},
)
config_setting(
  name = 'has_ubsan',
  values = {'define': 'have_ubsan=true'},
)

environment(name = "k8")
environment(name = "armhf-debian")
environment(name = "roborio")
environment(name = "cortex-m4f")
environment_group(
    name = "cpus",
    environments = [":k8", ":roborio", ":armhf-debian", ":cortex-m4f"],
    defaults = [":k8", ":roborio", ":armhf-debian"],
)
