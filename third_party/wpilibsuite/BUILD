licenses(['permissive'])

cc_library(
  name = 'ntcore',
  srcs = glob(['ntcore/src/**/*.cpp', 'ntcore/src/**/*.h']),
  hdrs = glob(['ntcore/include/**/*.h']),
  includes = ['ntcore/include'],
  visibility = ['//visibility:public']
)

cc_library(
  name = 'ni_libraries',
  srcs = [
    'allwpilib/ni-libraries/libFRC_NetworkCommunication.so.16.0.0',
    'allwpilib/ni-libraries/libRoboRIO_FRC_ChipObject.so.16.0.0',
    'allwpilib/ni-libraries/libNiFpgaLv.so.15.0.0',
    'allwpilib/ni-libraries/libNiFpga.so.15.0.0',
    'allwpilib/ni-libraries/libNiRioSrv.so.15.0.0',
    'allwpilib/ni-libraries/libspi.so.1.0.0',
    'allwpilib/ni-libraries/libi2c.so.1.0.0',
  ],
  linkstatic = 1
)

cc_library(
  name = 'hal',
  srcs = glob([
    'allwpilib/hal/lib/**/*.cpp', 'allwpilib/hal/lib/**/*.h'
  ]),
  hdrs = glob([
    'allwpilib/hal/include/**/*.h',
    'allwpilib/hal/include/**/*.hpp'
  ]),
  includes = [
    'allwpilib/hal/include',
    'allwpilib/hal/lib/athena/FRC_FPGA_ChipObject',
    'allwpilib/hal/lib/athena'
  ],
  linkopts = [
    '-lpthread',
    '-lm'
  ],
  deps = [':ni_libraries']
)

cc_library(
  name = 'wpilib',
  srcs = glob([
    'allwpilib/wpilibc/athena/src/**/*.cpp',
    'allwpilib/wpilibc/shared/src/**/*.cpp'
  ]),
  hdrs = glob([
    'allwpilib/wpilibc/athena/include/**/*.h',
    'allwpilib/wpilibc/athena/include/**/*.hpp',
    'allwpilib/wpilibc/shared/include/**/*.h',
    'allwpilib/wpilibc/shared/include/**/*.hpp',
    'allwpilib/wpilibc/shared/include/**/*.inc'
  ]),
  includes = [
    'allwpilib/wpilibc/athena/include',
    'allwpilib/wpilibc/shared/include'
  ],
  linkopts = [
    '-lpthread',
    '-ldl',
    '-lm'
  ],
  deps = [
    ':hal',
    ':ntcore'
  ],
  visibility = ['//visibility:public']
)
