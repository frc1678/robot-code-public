licenses(['permissive'])

cc_library(
  name = 'wpiutil',
  srcs = glob([
    'allwpilib/wpiutil/src/main/native/cpp/**/*.cpp',
    'allwpilib/wpiutil/src/main/native/cpp/**/*.h',
    'allwpilib/wpiutil/src/main/native/cpp/**/*.inc',
  ]),
  hdrs = glob([
    'allwpilib/wpiutil/src/main/native/include/**/*.h',
    'allwpilib/wpiutil/src/main/native/include/**/*.inl',
  ]),
  includes = [
    'allwpilib/wpiutil/src/main/native/include/',
  ]
)

cc_library(
  name = 'ntcore',
  srcs = glob([
    'allwpilib/ntcore/src/main/native/cpp/**/*.cpp',
    # 'ntcore/src/main/native/cpp/**/*.c',
    'allwpilib/ntcore/src/main/native/cpp/**/*.h',
  ], exclude = [
    'allwpilib/ntcore/src/main/native/cpp/jni/**/*',
  ]),
  hdrs = glob([
    'allwpilib/ntcore/src/main/native/include/**/*.h',
    'allwpilib/ntcore/src/main/native/include/**/*.inl',
  ]),
  includes = [
    'allwpilib/ntcore/src/main/native/include/',
  ],
  deps = [':wpiutil'],
  linkstatic = 1,
  visibility = ['//visibility:public']
)

cc_library(
  name = 'ni_libraries',
  srcs = glob(['allwpilib/ni-libraries/lib/*']),
  hdrs = glob([
    'allwpilib/ni-libraries/include/**/*.h',
  ]),
  includes = ['allwpilib/ni-libraries/include'],
  linkstatic = 1
)

cc_library(
  name = 'hal',
  srcs = glob([
    'allwpilib/hal/src/main/native/athena/**/*.cpp',
    'allwpilib/hal/src/main/native/athena/**/*.c',
    'allwpilib/hal/src/main/native/athena/**/*.h',
    'allwpilib/hal/src/main/native/cpp/cpp/*.cpp',
    'allwpilib/hal/src/main/native/cpp/handles/*.cpp',
    # 'allwpilib/hal/src/main/native/**/*.c',
    # 'allwpilib/hal/src/main/native/**/*.h',
  ]),
  hdrs = glob([
    'allwpilib/hal/src/main/native/include/**/*.h',
  ]),
  includes = [
    'allwpilib/hal/src/main/native/include/',
  ],
  linkopts = [
    '-lpthread',
    '-lm',
  ],
  deps = [':ni_libraries', ':wpiutil']
)

genrule(
  name = 'wpilib_version',
  srcs = [],
  outs = ['WPILibVersion.cpp'],
  cmd = """echo '
#include "WPILibVersion.h"
const char* GetWPILibVersion() {
  return "2018.1.1";
}
' > '$@'""",
)

cc_library(
  name = 'wpilib',
  srcs = glob([
    'allwpilib/wpilibc/src/main/native/cpp/**/*.cpp',
  ], exclude = [
    'allwpilib/wpilibc/src/main/native/cpp/CameraServer.cpp',
    'allwpilib/wpilibc/src/main/native/cpp/USBCamera.cpp',
    'allwpilib/wpilibc/src/main/native/cpp/vision/VisionRunner.cpp',
    'allwpilib/wpilibc/src/main/native/cpp/shuffleboard/*.cpp'
  ]) + ['WPILibVersion.cpp'],
  hdrs = glob([
    'allwpilib/wpilibc/src/main/native/include/**/*.h',
    'allwpilib/wpilibc/src/main/native/include/**/*.inc',
  ], exclude = [
    'allwpilib/wpilibc/src/main/native/include/CameraServer.h',
    'allwpilib/wpilibc/src/main/native/include/vision/VisionRunner.h',
    'allwpilib/wpilibc/src/main/native/include/frc/shuffleboard/*.h'
  ]),
  includes = [
    'allwpilib/wpilibc/src/main/native/include/',
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
  visibility = ['//visibility:public'],
  restricted_to = ['//tools:roborio'],
)
