load('//third_party/google/protobuf:protobuf.bzl', 'cc_proto_library')

cc_library(
  name = 'intake_queues',
  hdrs = ['queue_types.h'],
  deps = ['intake_proto', '//muan/proto:stack_proto', '//muan/queues:queues'],
  visibility = ['//visibility:public']
)

cc_proto_library(
  name = 'intake_proto',
  srcs = ['intake.proto'],
  protoc = '//third_party/google/protobuf:protoc',
  default_runtime = '//third_party/google/protobuf',
  visibility = ['//visibility:public']
)

cc_test(
  name = 'intake_test',
  srcs = ['intake_test.cpp'],
  deps = [
    '//muan/control:state_space',
    '//third_party/eigen',
    '//third_party/google/googletest:googletest'
  ]
)

cc_library(
  name = 'intake_controller',
  srcs = ['intake_controller.cpp'],
  hdrs = ['intake_controller.h'],
  deps = [
    '//muan/control:state_space',
    '//muan/units:units',
    '//third_party/eigen',
    ':intake_constants',
  ],
  visibility = ['//visibility:public']
)

cc_library(
  name = 'intake_constants',
  srcs = [':intake_constants.cpp'],
  hdrs = [':intake_constants.h'],
  deps = ['//third_party/eigen'],
)

genrule(
  name = 'intake_genrule',
  srcs = [],
  tools = [':intake_python'],
  cmd = '$(location :intake_python) $(location intake_constants.h) $(location intake_constants.cpp)',
  outs = ['intake_constants.h', 'intake_constants.cpp']
)

py_binary(
  name= 'intake_python',
  srcs = ['intake_controller.py'],
  main = 'intake_controller.py',
  deps = ['//muan/control:python_controls']
)
