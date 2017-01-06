package(default_visibility = ['//visibility:public'])
load('//third_party/google/protobuf:protobuf.bzl', 'cc_proto_library')

cc_library(
  name = 'math_utils',
  hdrs = ['math_utils.h'],
  srcs = ['math_utils.cpp'],
  deps = [
    '//muan/units:units',
    '//third_party/eigen'
  ],
  linkopts = ['-lm'],
  linkstatic = 1
)

cc_library(
  name = 'history',
  hdrs = ['history.h'],
  deps = [
    '//muan/units:units',
    '//third_party/aos/common:die'
  ],
  linkstatic = 1
)

cc_library(
  name = 'disk_brake',
  hdrs = ['disk_brake.h'],
  srcs = ['disk_brake.cpp'],
  deps = [
    '//muan/units:units',
    ':math_utils',
    '//third_party/aos/common:time'
  ]
)

cc_library(
  name = 'linear_interpolation',
  hdrs = ['linear_interpolation.h'],
  srcs = ['linear_interpolation.hpp'],
  deps = ['//third_party/aos/common:die']
)

cc_library(
  name = 'proto_utils',
  hdrs = [
    'proto_utils.h'
  ],
  srcs = [
    'proto_utils.cpp'
  ],
  deps = [
      '//third_party/google/protobuf',
      '//third_party/aos/common:die'
      ]
)

cc_proto_library(
  name = 'test_proto',
  srcs = ['test_proto.proto'],
  protoc = '//third_party/google/protobuf:protoc',
  default_runtime = '//third_party/google/protobuf'
)

cc_proto_library(
  name = 'test_death_proto',
  srcs = ['test_death_proto.proto'],
  protoc = '//third_party/google/protobuf:protoc',
  default_runtime = '//third_party/google/protobuf'
)

cc_test(
  name = 'proto_utils_test',
  srcs = ['proto_utils_test.cpp'],
  deps = [
    ':proto_utils',
    ':test_proto',
    ':test_death_proto',
    '//third_party/google/googletest'
  ]
)

cc_test(
  name = 'linear_interpolation_test',
  srcs = ['linear_interpolation_test.cpp'],
  deps = [
    ':linear_interpolation',
    ':disk_brake',
    '//third_party/google/googletest:googletest'
  ]
)

cc_test(
  name = 'history_test',
  srcs = ['history_test.cpp'],
  deps = [
    ':history',
    '//third_party/google/googletest:googletest'
  ]
)
