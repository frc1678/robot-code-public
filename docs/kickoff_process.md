# Kickoff Process

This page describes the process that you need to go through to set up our repo for another year of programming robots. This should be done around kickoff.

1. Copy generic robot
  * `cp -r generic_robot c####` where `####` is the year
2. Replace all instances of `generic_robot` with `c####`
  * `find ./c#### -type f -exec sed -i -e 's/generic_robot/c####/g; s/GENERIC_ROBOT/C####/g' {} \;`
3. Add `c####` to testing script
  * Add it to the `all_targets` list in `scripts/tests.py`
