load('//third_party/google/protobuf:protobuf.bzl', 'cc_proto_library')
package(default_visibility = ['//visibility:public'])

cc_proto_library(
  name = 'trigger_proto',
  srcs = ['trigger.proto'],
  protoc = '//third_party/google/protobuf:protoc',
  default_runtime = '//third_party/google/protobuf'
)

cc_library(
  name = 'trigger_mechanism',
  srcs = ['trigger_controller.cpp'],
  hdrs = ['trigger_controller.h'],
  deps = [
    '//muan/control:state_space',
    ':trigger_constants',
    ':trigger_queues',
    '//c2017/queue_manager:queue_manager'
  ]
)

cc_library(
  name = 'trigger_constants',
  srcs = [':trigger_constants.cpp'],
  hdrs = [':trigger_constants.h'],
  deps = ['//third_party/eigen']
)

cc_test(
  name = 'trigger_tests',
  srcs = ['trigger_tests.cpp'],
  deps = [
    '//third_party/google/googletest:googletest',
    ':trigger_mechanism'
  ]
)

genrule(
  name = 'trigger_genrule',
  srcs = [],
  tools = ['trigger_python'],
  cmd = '$(location :trigger_python) $(location trigger_constants.h) $(location trigger_constants.cpp)',
  outs = ['trigger_constants.h', 'trigger_constants.cpp']
)

py_binary(
  name = 'trigger_python',
  srcs = ['trigger.py'],
  main = 'trigger.py',
  deps = ['//muan/control:python_controls']
)

cc_library (
  name = 'trigger_queues',
  hdrs = ['queue_types.h'],
  deps = [
    'trigger_proto',
    '//muan/queues',
    '//muan/proto:stack_proto'
  ]
)

