load('@third_party//google/protobuf:protobuf.bzl', 'py_proto_library')

action_listener(
  name = 'compilation_database',
  mnemonics = [
    'CppCompile'
  ],
  extra_actions = [
    ':cpp_compilation_commands_gatherer'
  ],
  visibility = ['//visibility:public']
)

extra_action(
  name = 'cpp_compilation_commands_gatherer',
  tools = [':gather_compilation_commands'],
  out_templates = ['out_$(ACTION_ID).json'],
  cmd = '$(location :gather_compilation_commands) $(EXTRA_ACTION_FILE) $(output out_$(ACTION_ID).json)'
)

py_binary(
  name = 'gather_compilation_commands',
  srcs = ['gather_cdb.py'],
  deps = [':compile_info'],
  main = 'gather_cdb.py'
)

py_proto_library(
  name = 'compile_info',
  srcs = ['extra_actions.proto'],
  protoc = '@third_party//google/protobuf:protoc',
  default_runtime = '@third_party//google/protobuf:protobuf_python'
)
