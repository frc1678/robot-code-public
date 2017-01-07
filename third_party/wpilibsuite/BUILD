licenses(['permissive'])

cc_library(
  name = 'wpiutil',
  srcs = glob([
    'ntcore/wpiutil/src/**/*.cpp',
  ]),
  hdrs = glob([
    'ntcore/wpiutil/include/**/*.h',
  ]),
  includes = [
    'ntcore/wpiutil/include',
  ]
)

cc_library(
  name = 'ntcore',
  srcs = glob([
    'ntcore/src/**/*.cpp',
    'ntcore/src/**/*.h',
  ]),
  hdrs = glob([
    'ntcore/include/**/*.h',
  ]),
  includes = [
    'ntcore/include',
    'ntcore/include/support',
  ],
  deps = [':wpiutil'],
  linkstatic = 1,
  visibility = ['//visibility:public']
)

cc_library(
  name = 'ni_libraries',
  srcs = [ 'allwpilib/ni-libraries/lib/%s' % s for s in [
    'libFRC_NetworkCommunication.so.17.0.0',
    'libRoboRIO_FRC_ChipObject.so.17.0.0',
    'libNiFpgaLv.so.16.0.0',
    'libNiFpga.so.16.0.0',
    'libNiRioSrv.so.16.0.0',
    'libspi.so.1.0.0',
    'libi2c.so.2.0.0',
    'libvisa.so',
  ]],
  hdrs = glob([
    'allwpilib/ni-libraries/include/**/*.h',
  ]),
  includes = ['allwpilib/ni-libraries/include'],
  linkstatic = 1
)

cc_library(
  name = 'hal',
  srcs = glob([
    'allwpilib/hal/lib/**/*.cpp',
    'allwpilib/hal/lib/**/*.h',
    'allwpilib/ntcore/wpiutil/include/**/*.h',
  ]),
  hdrs = glob([
    'allwpilib/hal/include/**/*.h',
    'allwpilib/hal/include/**/*.hpp',
  ]),
  includes = [
    'allwpilib/hal/include',
    'allwpilib/hal/lib/athena/FRC_FPGA_ChipObject',
    'allwpilib/hal/lib/athena',
    'ntcore/wpiutil/include',
  ],
  linkopts = [
    '-lpthread',
    '-lm',
  ],
  deps = [':ni_libraries', ':wpiutil']
)

cc_library(
  name = 'wpilib',
  srcs = glob([
    'allwpilib/wpilibc/athena/src/**/*.cpp',
    'allwpilib/wpilibc/shared/src/**/*.cpp',
  ], exclude = [
    'allwpilib/wpilibc/athena/src/vision/*',
    'allwpilib/wpilibc/athena/src/CameraServer.cpp',
    'allwpilib/wpilibc/athena/src/USBCamera.cpp',
  ]),
  hdrs = glob([
    'allwpilib/wpilibc/athena/include/**/*',
    'allwpilib/wpilibc/shared/include/**/*',
  ], exclude = [
    'allwpilib/wpilibc/athena/src/Vision/*',
  ]),
  includes = [
    'allwpilib/wpilibc/athena/include',
    'allwpilib/wpilibc/shared/include',
  ],
  linkopts = [
    '-lpthread',
    '-ldl',
    '-lm',
  ],
  deps = [
    ':hal',
    ':ntcore',
  ],
  visibility = ['//visibility:public']
)
