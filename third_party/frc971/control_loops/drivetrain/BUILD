package(default_visibility = ['//visibility:public'])
licenses(['notice'])

load('//third_party/google/protobuf:protobuf.bzl', 'cc_proto_library')

cc_proto_library(
  name = 'drivetrain_protos',
  srcs = ['drivetrain.proto'],
  protoc = '//third_party/google/protobuf:protoc',
  default_runtime = '//third_party/google/protobuf'
)

cc_library(
  name = 'drivetrain_queues',
  hdrs = ['queue_types.h'],
  deps = [
    ':drivetrain_protos',
    '//muan/proto:stack_proto',
    '//muan/queues',
  ],
)

cc_library(
  name = 'gear',
  hdrs = ['gear.h'],
  deps = [':drivetrain_queues'],
)

cc_library(
  name = 'drivetrain_config',
  hdrs = [
    'drivetrain_config.h',
  ],
  deps = [
    '//third_party/frc971/control_loops:state_feedback_loop',
    '//third_party/frc971:shifter_hall_effect',
  ],
)

cc_library(
  name = 'ssdrivetrain',
  srcs = [
    'ssdrivetrain.cc',
  ],
  hdrs = [
    'ssdrivetrain.h',
  ],
  deps = [
    ':drivetrain_queues',
    ':drivetrain_config',
    '//third_party/aos/common/controls:polytope',
    '//third_party/aos/common/util:trapezoid_profile',
    '//third_party/aos/common:math',
    '//third_party/frc971/control_loops:coerce_goal',
    '//third_party/frc971/control_loops:state_feedback_loop',
    '//third_party/frc971/control_loops/paths:trajectory',
  ],
)

cc_library(
  name = 'polydrivetrain',
  srcs = [
    'polydrivetrain.cc',
  ],
  hdrs = [
    'polydrivetrain.h',
  ],
  deps = [
    ':drivetrain_queues',
    ':drivetrain_config',
    ':gear',
    '//third_party/aos/common/controls:polytope',
    '//third_party/aos/common:math',
    '//third_party/frc971/control_loops:state_feedback_loop',
    '//third_party/frc971/control_loops:coerce_goal',
    '//third_party/frc971:shifter_hall_effect',
  ],
)

cc_library(
  name = 'drivetrain_lib',
  srcs = [
    'drivetrain.cc',
  ],
  hdrs = [
    'drivetrain.h',
  ],
  deps = [
    ':drivetrain_queues',
    ':polydrivetrain',
    ':ssdrivetrain',
    '//muan/wpilib/gyro:gyro_queues',
    '//muan/wpilib:wpilib_queues'
  ],
)

cc_library(
  name = 'drivetrain_simulation',
  srcs = [
    'drivetrain_simulation.cc'
  ],
  hdrs = [
    'drivetrain_simulation.h',
  ],
  deps = [
    '//third_party/google/googletest:googletest',
    ':drivetrain_queues',
    ':drivetrain_lib',
    ':drivetrain_config',
    '//third_party/frc971/control_loops:state_feedback_loop',
    '//muan/wpilib/gyro:gyro_queues',
    '//muan/logging:logger',
    '//muan/queues:queue_manager',
  ],
)

cc_test(
  name = 'drivetrain_lib_test',
  srcs = [
    'drivetrain_lib_test.cc',
  ],
  deps = [
    '//third_party/google/googletest:googletest',
    ':drivetrain_simulation',
    ':drivetrain_queues',
    ':drivetrain_lib',
    ':drivetrain_config',
    '//third_party/frc971/control_loops:state_feedback_loop',
    '//muan/wpilib/gyro:gyro_queues',
    '//third_party/frc971/control_loops/drivetrain/y2016:polydrivetrain_plants',
    '//third_party/frc971/control_loops/drivetrain/y2016:drivetrain_base',
  ],
)
