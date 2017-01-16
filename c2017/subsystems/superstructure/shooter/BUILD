load('//third_party/google/protobuf:protobuf.bzl', 'cc_proto_library')
package(default_visibility = ['//visibility:public'])

cc_library(
  name = 'shooter_queues',
  hdrs = ['queue_types.h'],
  deps = ['shooter_proto', '//muan/queues', '//muan/proto:stack_proto']
)

cc_proto_library(
  name = 'shooter_proto',
  srcs = ['shooter.proto'],
  protoc = '//third_party/google/protobuf:protoc',
  default_runtime = '//third_party/google/protobuf'
)

cc_library(
  name = 'shooter_controller',
  srcs = ['shooter_controller.cpp'],
  hdrs = ['shooter_controller.h'],
  deps = ['//muan/control:state_space', '//muan/wpilib:wpilib_queues', ':shooter_constants', ':shooter_queues']
)

cc_test(
  name = 'shooter_controller_test',
  srcs = ['shooter_controller_test.cpp'],
  deps = [
    ':shooter_controller',
    '//muan/control:state_space',
    '//muan/wpilib:wpilib_queues',
    '//third_party/google/googletest:googletest',
    ':shooter_constants',
    ':shooter_queues',
    ':shooter_proto'
  ]
)

cc_library(
  name = 'shooter_constants',
  srcs = [':shooter_constants.cpp'],
  hdrs = [':shooter_constants.h'],
  deps = ['//third_party/eigen']
)

genrule(
  name = 'shooter_genrule',
  srcs = [],
  tools = ['shooter_python'],
  cmd = '$(location :shooter_python) $(location shooter_constants.h) $(location shooter_constants.cpp)',
  outs = ['shooter_constants.h', 'shooter_constants.cpp']
)

py_binary(
  name = 'shooter_python',
  srcs = ['shooter_controller.py'],
  main = 'shooter_controller.py',
  deps = ['//muan/control:python_controls']
)
