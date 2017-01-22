load('//third_party/google/protobuf:protobuf.bzl', 'cc_proto_library')

cc_library(
  name = 'wpilib_queues',
  hdrs = [
    'queue_types.h',
  ],
  deps = [
    ':wpilib_proto',
    '//muan/proto:stack_proto',
    '//muan/queues:queues'
  ],
  visibility = ['//visibility:public']
)

cc_proto_library(
  name = 'wpilib_proto',
  srcs = ['wpilib_superstructure.proto'],
  protoc = '//third_party/google/protobuf:protoc',
  default_runtime = '//third_party/google/protobuf',
  visibility = ['//visibility:public']
)

cc_library(
  name = 'superstructure_interface',
  srcs = [
    'superstructure_interface.cpp',
  ],
  hdrs = [
    'superstructure_interface.h',
  ],
  deps = [
    '//c2017/queue_manager',
    '//muan/utils:math_utils',
    '//muan/wpilib:pcm_wrapper',
    '//muan/wpilib:can_wrapper',
    '//third_party/wpilibsuite:wpilib'
  ],
  visibility = ['//visibility:public']
)

cc_library(
  name = 'wpilib_interface',
  srcs = [
    'wpilib_interface.cpp',
  ],
  hdrs = [
    'wpilib_interface.h',
  ],
  deps = [
    '//c2017/queue_manager',
    '//muan/wpilib:can_wrapper',
    '//third_party/wpilibsuite:wpilib',
    '//muan/wpilib/gyro:gyro',
    ':drivetrain_interface',
    ':superstructure_interface',
  ],
  visibility = ['//visibility:public']
)

cc_library(
  name = 'drivetrain_interface',
  srcs = [
    'drivetrain_interface.cpp'
  ],
  hdrs = [
    'drivetrain_interface.h'
  ],
  deps = [
    '//c2017/queue_manager',
    '//muan/wpilib/gyro:gyro',
    '//third_party/wpilibsuite:wpilib',
    '//muan/utils:math_utils'
  ]
)
