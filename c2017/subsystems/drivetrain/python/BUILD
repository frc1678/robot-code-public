package(default_visibility = ['//visibility:public'])
licenses(['notice'])

py_binary(
  name = 'drivetrain',
  srcs = [
    'drivetrain.py',
  ],
  deps = [
    # '//external:python-gflags',
    # '//external:python-glog',
    '//third_party/frc971/control_loops/python:controls',
  ],
)

py_binary(
  name = 'polydrivetrain',
  srcs = [
    'polydrivetrain.py',
    'drivetrain.py',
  ],
  deps = [
    # '//external:python-gflags',
    # '//external:python-glog',
    '//third_party/frc971/control_loops/python:controls',
  ],
)

py_library(
  name = 'polydrivetrain_lib',
  srcs = [
    'polydrivetrain.py',
    'drivetrain.py',
  ],
  deps = [
    # '//external:python-gflags',
    # '//external:python-glog',
    '//third_party/frc971/control_loops/python:controls',
  ],
)
