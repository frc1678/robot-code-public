licenses(['permissive'])

load('//third_party/google/protobuf:protobuf.bzl', 'py_proto_library')

py_proto_library(
  name = 'compile_info',
  srcs = ['src/main/protobuf/extra_actions.proto'],
  protoc = '//third_party/google/protobuf:protoc',
  default_runtime = '//third_party/google/protobuf:protobuf_python',
  visibility = ['//visibility:public']
)
