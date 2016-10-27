cc_test(
  name = 'scoop_test',
  srcs = ['scoop_test.cpp'],
  deps = [
    ':scoop',
    '//muan/control:state_space',
    '//third_party/eigen',
    '//third_party/google/googletest:googletest'
  ]
)

cc_library(
  name = 'scoop',
  srcs = ['scoop.cpp'],
  hdrs = ['scoop.h'],
  deps = [
    ':scoop_constants',
    '//muan/control:state_space',
    '//muan/units:units',
    '//third_party/eigen'
  ],
  visibility = ['//visibility:public']
)

cc_library(
  name = 'scoop_constants',
  srcs = ['scoop_constants.cpp'],
  hdrs = ['scoop_constants.h'],
  deps = ['//third_party/eigen']
)

genrule(
  name = 'scoop_genrule',
  srcs = [],
  tools = [':scoop_python'],
  cmd = '$(location :scoop_python) $(location scoop_constants.h) $(location scoop_constants.cpp)',
  outs = ['scoop_constants.h', 'scoop_constants.cpp']
)

py_binary(
  name = 'scoop_python',
  srcs = ['scoop.py'],
  main = 'scoop.py',
  deps = ['//muan/control:python_controls']
)
