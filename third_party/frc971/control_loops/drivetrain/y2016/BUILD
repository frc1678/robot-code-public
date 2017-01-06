package(default_visibility = ['//visibility:public'])
licenses(['notice'])

genrule(
  name = 'genrule_drivetrain',
  visibility = ['//visibility:private'],
  cmd = '$(location //third_party/frc971/control_loops/drivetrain/y2016/python:drivetrain) $(OUTS)',
  tools = [
    '//third_party/frc971/control_loops/drivetrain/y2016/python:drivetrain',
  ],
  outs = [
    'drivetrain_dog_motor_plant.h',
    'drivetrain_dog_motor_plant.cc',
    'kalman_drivetrain_motor_plant.h',
    'kalman_drivetrain_motor_plant.cc',
  ],
)

genrule(
  name = 'genrule_polydrivetrain',
  visibility = ['//visibility:private'],
  cmd = '$(location //third_party/frc971/control_loops/drivetrain/y2016/python:polydrivetrain) $(OUTS)',
  tools = [
    '//third_party/frc971/control_loops/drivetrain/y2016/python:polydrivetrain',
  ],
  outs = [
    'polydrivetrain_dog_motor_plant.h',
    'polydrivetrain_dog_motor_plant.cc',
    'polydrivetrain_cim_plant.h',
    'polydrivetrain_cim_plant.cc',
  ],
)

cc_library(
  name = 'polydrivetrain_plants',
  srcs = [
    'polydrivetrain_dog_motor_plant.cc',
    'drivetrain_dog_motor_plant.cc',
    'kalman_drivetrain_motor_plant.cc',
  ],
  hdrs = [
    'polydrivetrain_dog_motor_plant.h',
    'drivetrain_dog_motor_plant.h',
    'kalman_drivetrain_motor_plant.h',
  ],
  deps = [
    '//third_party/frc971/control_loops:state_feedback_loop',
  ],
)

cc_library(
  name = 'drivetrain_base',
  srcs = [
    'drivetrain_base.cc',
  ],
  hdrs = [
    'drivetrain_base.h',
  ],
  deps = [
    ':polydrivetrain_plants',
    '//third_party/frc971/control_loops/drivetrain:drivetrain_config',
    # '//third_party/frc971:shifter_hall_effect',
    # '//third_party/frc971/control_loops/drivetrain/y2016:constants',
  ],
)

cc_binary(
  name = 'drivetrain',
  srcs = [
    'drivetrain_main.cc',
  ],
  deps = [
    ':drivetrain_base',
    '//third_party/aos/linux_code:init',
    '//third_party/frc971/control_loops/drivetrain:drivetrain_lib',
  ],
)
