package(default_visibility = ['//visibility:public'])
load('//third_party/google/protobuf:protobuf.bzl', 'cc_proto_library')

cc_library(
  name = 'math_utils',
  hdrs = ['math_utils.h'],
  deps = ['//muan/units:units', '//third_party/eigen'],
  linkstatic = 1
)

cc_library(
  name = 'history',
  hdrs = ['history.h'],
  deps = ["//muan/units:units"],
  linkstatic = 1
)

cc_library(
  name = 'timing',
  hdrs = [
    'timing_utils.h',
    'timer.h'
  ],
  srcs = [
    'timing_utils.cpp',
    'timer.cpp'
  ],
  deps = ["//muan/units:units"]
)

cc_library(
  name = 'proto_utils',
  hdrs = [
    'proto_utils.h'
  ],
  srcs = [
    'proto_utils.cpp'
  ],
  deps = ['//third_party/google/protobuf']
)

cc_proto_library(
  name = 'test_proto',
  srcs = ['test_proto.proto'],
  protoc = '//third_party/google/protobuf:protoc',
  default_runtime = '//third_party/google/protobuf'
)

cc_test(
  name = 'proto_utils_test',
  srcs = ['proto_utils_test.cpp'],
  deps = [
    ':proto_utils',
    ':test_proto',
    '//third_party/google/googletest'
  ]
)
cc_test(
  name = 'utils_test',
  srcs = ['utils_test.cpp'],
  deps = [
    ':timing',
    'history',
    '//third_party/google/googletest:googletest'
  ]
)
