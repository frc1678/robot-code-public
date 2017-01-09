#!/usr/bin/python

import sys

input_str = sys.stdin.read()

if input_str == "test":
    print("test_passed")

print(" ".join(sys.argv))

sys.exit(0)
