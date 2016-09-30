load('/third_party/google/protobuf/protobuf', 'cc_proto_library')

cc_library(
  name = 'drivetrain',
  srcs = ['drivetrain_subsystem.cpp'],
  hdrs = ['drivetrain_subsystem.h'],
  deps = [':drivetrain_constants'],
  visibility = ['//visibility:public']
)

cc_library(
  name = 'drivetrain_constants',
  srcs = ['drivetrain_constants.cpp'],
  hdrs = ['drivetrain_constants.h'],
  deps = ['//third_party/eigen'],
  linkstatic=1
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

cc_proto_library(
  name = 'drivetrain_protos',
  srcs = ['drivetrain.proto'],
  protoc = '//third_party/google/protobuf:protoc',
  default_runtime = '//third_party/google/protobuf'
)
