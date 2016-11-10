cc_test(
  name = 'stop_test',
  srcs = ['stop_test.cpp'],
  deps = [
    ':stop',
    '//muan/control:state_space',
    '//third_party/eigen',
    '//third_party/google/googletest:googletest'
  ]
)

cc_library(
  name = 'stop',
  srcs = ['stop.cpp', 'stop_pid.cpp'],
  hdrs = ['stop.h', 'stop_pid.h'],
  deps = [
    ':stop_constants',
    '//muan/control:state_space',
    '//muan/control:pid_controller',
    '//muan/units:units',
    '//third_party/eigen'
  ],
  visibility = ['//visibility:public']
)

cc_library(
  name = 'stop_constants',
  srcs = ['stop_constants.cpp'],
  hdrs = ['stop_constants.h'],
  deps = ['//third_party/eigen']
)

genrule(
  name = 'stop_genrule',
  srcs = [],
  tools = [':stop_python'],
  cmd = '$(location :stop_python) $(location stop_constants.h) $(location stop_constants.cpp)',
  outs = ['stop_constants.h', 'stop_constants.cpp']
)

py_binary(
  name = 'stop_python',
  srcs = ['stop.py'],
  main = 'stop.py',
  deps = ['//muan/control:python_controls']
)
