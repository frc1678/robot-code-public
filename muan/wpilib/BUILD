package(default_visibility = ['//visibility:public'])
load('//third_party/google/protobuf:protobuf.bzl', 'cc_proto_library')

cc_library(
  name = 'can_wrapper',
  srcs = ['can_wrapper.cpp'],
  hdrs = ['can_wrapper.h'],
  deps = [
    '//third_party/aos/linux_code:init',
    '//third_party/aos/common/util:phased_loop',
    '//third_party/aos/common:time',
    ':pdp_wrapper',
    ':pcm_wrapper',
  ]
)

cc_library(
  name = 'pdp_wrapper',
  srcs = ['pdp_wrapper.cpp'],
  hdrs = ['pdp_wrapper.h'],
  deps = [
    '//third_party/wpilibsuite:wpilib',
    '//muan/queues',
    ':state_protos',
    '//muan/proto:stack_proto',
    '//third_party/aos/common:die',
  ]
)

cc_library(
  name = 'pcm_wrapper',
  srcs = ['pcm_wrapper.cpp'],
  hdrs = ['pcm_wrapper.h'],
  deps = [
    '//third_party/wpilibsuite:wpilib',
    '//third_party/aos/common:die',
  ]
)

cc_binary(
  name = 'pdp_test',
  srcs = ['pdp_test.cpp'],
  deps = [':can_wrapper',
          '//third_party/wpilibsuite:wpilib']
)

cc_library(
  name = 'motor_safety',
  srcs = ['motor_safety.cpp'],
  hdrs = ['motor_safety.h'],
  deps = ['//muan/utils:history']
)

cc_test(
  name = 'motor_safety_test',
  srcs = ['motor_safety_test.cpp'],
  deps = [':motor_safety', '//third_party/google/googletest:googletest'],
  linkopts = ['-lm']
)

cc_binary(
  name = 'pcm_test',
  srcs = ['pcm_test.cpp'],
  deps = [':can_wrapper',
          '//third_party/wpilibsuite:wpilib']
)

cc_library(
  name = 'wpilib_queues',
  hdrs = ['queue_types.h'],
  deps = [
    '//muan/proto:stack_proto',
    '//muan/queues',
    ':state_protos'
  ]
)

cc_proto_library(
  name = 'state_protos',
  srcs = ['state.proto'],
  protoc = '//third_party/google/protobuf:protoc',
  default_runtime = '//third_party/google/protobuf'
)
