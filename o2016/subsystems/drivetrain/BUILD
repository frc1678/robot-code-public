cc_library(
  name = 'drivetrain_constants',
  srcs = ['drivetrain_constants.cpp'],
  hdrs = ['drivetrain_constants.h'],
  deps = ['//third_party/eigen']
)

genrule(
  name = 'drivetrain_genrule',
  srcs = [],
  tools = [':drivetrain_python'],
  cmd = '$(location :drivetrain_python) $(location drivetrain_constants.h) $(location drivetrain_constants.cpp)',
  outs = ['drivetrain_constants.h', 'drivetrain_constants.cpp']
)

py_binary(
  name = 'drivetrain_python',
  srcs = ['drivetrain.py'],
  main = 'drivetrain.py',
  deps = ['//muan/control:python_controls']
)
