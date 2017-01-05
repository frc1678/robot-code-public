cc_library(
  name = 'drivetrain_action',
  srcs = ['drivetrain_action.cpp'],
  hdrs = ['drivetrain_action.h'],
  deps = ['//third_party/frc971/control_loops/drivetrain:drivetrain_queues'],
  visibility = ['//visibility:public']
)

cc_test(
  name = 'drivetrain_action_test',
  srcs = ['drivetrain_action_test.cpp'],
  deps = [':drivetrain_action', '//third_party/google/googletest']
)
