#!/usr/bin/env python

import subprocess
import argparse

# Array of arrays representing targets.
# Inner arrays in the format of:
# [short name, bazel target, should test, should build for roborio]
all_targets = [
    ["muan", "//muan/...", True, True],
    ["c2017", "//c2017/...", True, True],
    ["o2017", "//o2017/...", True, True],
    ["testbench", "//testbench/...", False, True],
    ["generic_robot", "//generic_robot/...", True, True],
    ["drivetrain", "//third_party/frc971/control_loops/drivetrain:drivetrain_lib_test", False, True],
    ["o2016", "//o2016/...", True, True],
]

def print_colored(text):
    """
    Prints text in a bright color, to get the user's attention
    """
    print("\033[93m{}\033[0m".format(text))

def get_exclude_string(target, exclude):
    """
    Returns all rules in the bazel target that depend on exclude, in a format
    that is useful for being passed to a bazel test command

    For example, using `//third_party/wpilibsuite:wpilib` as exclude will return
    all the rules in `target` that depend on wpilib, in the format:

        `-//target:one -//target/foo:bar -//target/something:else`
    """
    command = "bazel query 'rdeps({}, {})'".format(target, exclude)
    children = subprocess.check_output(command, shell=True).decode("utf-8").split("\n")
    return " ".join(["-" + child for child in children[:-1]])

def main():
    parser = argparse.ArgumentParser(description="Test all of the code in the repo")
    parser.add_argument("--no-vision", action="store_true", help="Disable testing code that uses vision.\n"
                                                                 "Used on computers that do not have opencv installed.")
    parser.add_argument("--only", help="A comma separated list of targets that should be the only targets tested.")
    parser.add_argument("--list", action="store_true", help="List all targets and exit.")
    args = parser.parse_args()

    if args.list:
        print("Available targets:\n" + "\n".join([target[0] for target in all_targets]))
        exit()

    if args.only == None:
        targets = all_targets
        print_colored("Testing all targets")
    else:
        targets = []
        for target in all_targets:
            if target[0] in [option.strip() for option in args.only.split(",")]:
                targets.append(target)
        print_colored("Testing only the following targets:\n" + "\n".join([target[0] for target in targets]))

    for target in targets:
        vision_exclude_string = get_exclude_string(target[1], "//muan/vision")
        vision_exclude_string += " "
        vision_exclude_string += get_exclude_string(target[1], "//muan/vision:video_stream")
        if args.no_vision:
            vision_excludes = vision_exclude_string
        else:
            vision_excludes = ""

        print_colored("Testing {}".format(target[0]))

        if target[2]: # Test target
            wpilib_excludes = get_exclude_string(target[1], "//third_party/wpilibsuite:wpilib")
            bazel_command = "bazel test --test_output=errors {} -- {}".format(target[1], wpilib_excludes + " " + vision_excludes)
            print_colored("Running command: {}".format(bazel_command))
            subprocess.check_call(bazel_command, shell=True)
        if target[3]: # Build target
            bazel_command = "bazel build {} --cpu=roborio -- {}".format(target[1], vision_exclude_string)
            print_colored("Running command: {}".format(bazel_command))
            subprocess.check_call(bazel_command, shell=True)

    subprocess.check_call("./scripts/cpplint/run-cpplint.sh", shell=True)
    subprocess.check_call("./scripts/check-format.py --no-fail", shell=True)

if __name__ == "__main__":
    main()
