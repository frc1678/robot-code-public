#lemonscript-transpiler [![Build Status](https://travis-ci.org/WesleyAC/lemonscript-transpiler.svg)](https://travis-ci.org/WesleyAC/lemonscript-transpiler) [![Coverage Status](https://coveralls.io/repos/github/WesleyAC/lemonscript-transpiler/badge.svg?branch=master)](https://coveralls.io/github/WesleyAC/lemonscript-transpiler?branch=master) [![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE.md)

This is a script that converts .func files into a .h and .cpp file to be used with lemonscript.

##`.func` file format

`transpile.py` takes in a directory containing all the `.func` files that you want to compile, and outputs a `auto_functions.cpp` and `auto_functions.h` file to be used in the robot code. Here's an example of a valid `.func` file:

```
Wait(float time)

include {
  "muan/utils/timing_utils.h"
}

global {
  float start_time
}

init {
  start_time = muan::now();
  return false;
}

periodic {
  return (muan::now() - start_time > time);
}
```

A few things to keep in mind:

* The first line of the file must have the name of the auto function, and all of it's arguments on it.
* You cannot have indentation before `include`, `global`, `init`, or `periodic`.
* All files to include must be on a different line in the `include` block.
* If you try to break the transpiler, you will be able to. It should handle any reasonable file, but don't try to break it.

##Supported units

Lemonscript treats all arguments as void pointers, and internally converts them to the correct units. Because of this, only the following types are supported as arguments to auto functions:

* `int`
* `bool`
* `float`
* `std::string`

##Arguments

`transpile.py` has the following flags:

| flag              | function |
| ----------------- | -------- |
| `--input-files`   | A list of `.func` files to compile. |
| `--output-header` | A name of a file to put the transpiled .cpp code in. |
| `--output-cpp`    | A name of a file to put the transpiled .h code in. |
| `--format`        | Use `clang-format` to make the generated code pretty. |
| `-v`              | Sets log level to be more verbose. You may use more than one, but the default is everything but debug, so that's pretty useless. |
| `-q`              | Sets log level to be less verbose. You may use more than one. For example `-qq` will hide everything except errors. `-qqq` **should** result in no output to stdout. |

##Testing

To simply run the tests:

```
py.test ./tests
```

If you want coverage information as well:

```
coverage run -m py.test ./tests/
```
