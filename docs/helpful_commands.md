# Helpful Commands You May Forget

## Writing Code

`bazel build //o2017:frc1678 --cpu=roborio`:
Use this to build the code and check for errors. Replace o2017 with whatever year you want.

`bazel run //o2017:deploy --cpu=roborio -- --team=1678`:
Deploys code onto specified robot. YOU MUST BE CONNECTED TO THE ROBOT’S WIFI! If you’re asked for a password, hit enter.

`bazel test //o2017/subsystems/superstructure/your_mechanism:all`:
Use this to test just your subsystem.

`./scripts/tests.py`:
This is what Jenkins does- builds and test all directories in the robot-code folder. Also checks formatting. Takes a long time!

`clang-format -i your-file.cpp`:
Run Clang-Format on a specific file. You can include a path if the target file isn’t in your current directory.

## Networking and Debugging

`ssh admin@roborio-1678-frc.local`:
Puts you inside the robot

`nmap -sn 10.16.78.0-255`:
Lists what is connected to the robot. Helpful to see whether the roborio / the jetson aren’t connected or they’re just on an unexpected IP address.

`ss -tupa`:
List network connections. Useful to see whether a server or client is running correctly.

`top | grep program-name`:
Check if program-name is running and view memory and CPU usage.

## Git

`git remote add teammate https://github.com/teammate/robot-code`:
Add a teammate's fork as a remote so you can build and edit their WIP code.

`git stash`:
Move changes to the stash. Useful if you want to switch branches without
committing or if you find yourself working on the wrong branch. Apply changes
using `git stash pop`.
