load('//third_party/google/protobuf:protobuf.bzl', 'cc_proto_library')

cc_library(
  name = 'pcm_wrapper',
  srcs = ['pcm_wrapper.cpp'],
  hdrs = ['pcm_wrapper.h'],
  deps = [
    '//third_party/wpilibsuite:wpilib',
    '//third_party/aos/common:die',
  ]
)

cc_proto_library(
  name = 'state_protos',
  srcs = ['state.proto'],
  protoc = '//third_party/google/protobuf:protoc',
  default_runtime = '//third_party/google/protobuf'
)
