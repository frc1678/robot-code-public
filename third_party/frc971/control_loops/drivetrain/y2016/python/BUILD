package(default_visibility = ['//visibility:public'])
licenses(['notice'])

py_binary(
  name = 'drivetrain',
  srcs = [
    'drivetrain.py',
  ],
  deps = [
    '//third_party/frc971/control_loops/python:drivetrain',
  ],
)

py_binary(
  name = 'polydrivetrain',
  srcs = [
    'polydrivetrain.py',
    'drivetrain.py',
  ],
  deps = [
    '//third_party/frc971/control_loops/python:polydrivetrain',
  ],
)

py_library(
  name = 'polydrivetrain_lib',
  srcs = [
    'polydrivetrain.py',
    'drivetrain.py',
  ],
  deps = [
    '//third_party/frc971/control_loops/python:controls',
  ],
)
