# 1678 Robot Code
[![Buildkite](https://badge.buildkite.com/f8ab8cc73e5ea894cabdbf5d53b97cbaee3665dc043dbc2231.svg)](https://buildkite.com/citrus-circuits/robot-code)

This repository holds all of our robot code, starting from the 2016 offseason.

## Structure

* `third_party`: Code written by people not on our team.
* `muan`: Code that is shared between all robots.
* `tools`: Things that are used to build our code.

The only one of those three that you are likely to care about is `muan`.

The code for specific robots is in a directory starting with either the letter "o" for offseason or "c" for competition, and ending with the year that that robot is from. For example, `o2016` is our incredibly cute 2016 offseason robot.

## Building the Code

We use [bazel](https://bazel.io) to build our code. You must be running linux to use bazel. A typical bazel build command would look like this:

```
bazel build //o2016 --cpu=roborio
```

You should also test anything that you change:

```
bazel test //c2018/submarine_subsystem/...
```

Before you push your code, you can run our automated test script. This is what Jenkins CI runs, so if you don't like failed Jenkins builds you should always run it before pushing:

```
./scripts/tests.py
```

## Style Guide

We use the [Google C++ style guide](https://google.github.io/styleguide/cppguide.html) (mostly). You should **read all of it and make sure that your code adheres to it**. There are a few parts that we do not follow, and are aware of:

* We use `.cpp` as the source file extension
* We do make use of templates in some places

In addition to the Google C++ style guide, you should also make sure that you are writing good commit messages! Here are a couple articles on this: [1](http://tbaggery.com/2008/04/19/a-note-about-git-commit-messages.html) [2](http://chris.beams.io/posts/git-commit/).

Particularly, you should make sure to write your commits in the imperative mood ("Add thingamadoodle", not "Added thingamadoodle") and capitalize the first word. If the change is complicated, use multiple lines to describe it.

Additionally, you should try to make your commits as small as possible (often referred to as making "atomic" commits). This helps other people understand what you are doing.

We also use bazel's [Buildifier](https://github.com/bazelbuild/buildifier) to format BUILD files. The formatter
establishes quite a few
[conventions](https://github.com/bazelbuild/bazel/blob/master/site/versions/master/docs/skylark/build-style.md).
Highlights include:

 - Use four spaces for indenting
 - Use double quotes (") instead of single quotes (')

We use SI units and [radians](http://math.rice.edu/~pcmi/sphere/drg_txt.html).

## Contributing

Here's how to get your code into the main robot repository:

### If you've just joined the team:
1. Make an account on [GitHub](https://github.com/).
2. Ask the robot programming lead to add your account to the frc1678 robot programming team.

### If it's the first time you've contributed to this repo:
1. Fork this repo
  1. Log onto github and navigate to the repo [robot-code](https://github.com/frc1678/robot-code).
  2. Click the "Fork" button in the upper right corner of the screen.
2. Clone your forked repo.
  * `git clone https://github.com/<your_name>/robot-code.git`, where <your_name> is your github username.

### Any time you want to make a change:
1. Create and checkout a new branch.
  * `git checkout -b <your_branch_name>`, where <your_branch_name> is a descriptive name for your branch. For example `fix-shooter-wheel`, `two-ball-auto`, or `climbing`. Use dashes in the branch name, not underscores.
2. Make whatever code changes you want/need/ to make. Be sure to write tests for your changes!
3. Commit your work locally.
  * Try to make your commits as atomic (small) as possible. For example, moving functions around should be different from adding features, and changes to one subsystem should be in a different commit than changes to another subsystem.
  * Follow [these](http://tbaggery.com/2008/04/19/a-note-about-git-commit-messages.html) conventions for commit messages. Or else.
  * If your change is anything more than a few lines or small fixes, don't skip the extended description. If you are always using `git commit` with the `-m` option, stop doing that.
4. Push to your forked repo.
  * `git push origin <your_branch_name>`.
5. Submit a pull request.
  1. Log into github.
  2. Go to the page for your forked repo.
  3. Select the branch that you just pushed from the "Branch" dropdown menu.
  4. Click "New Pull Request".
  5. Review the changes that you made.
  6. If you are happy with your changes, click "Create Pull Request".
6. Wait
  * People must review (and approve of) your changes before they are merged.
    * Specifically, the rules are that one of the following two conditions must be true for it to get merged:
      1. 1 mentor and 1 other person have approved
      2. 2 experienced students and one other person have approved
  * If there are any concerns about your pull request, fix them. Depending on how severe the concerns are, the pull request may be merged without it, but everyone will be happier if you fix your code. To update your PR, just push to the branch on your forked repo.
  * Don't dismiss someone's review when you make changes - instead, ask them to re-review it.
7. Merge your changes into master
  * If there are no conflicts, push the "Squash and merge" button, write a good commit message, and merge the changes.
  * If there are conflicts, fix them locally on your branch, push them, wait for Jenkins to pass, and then squash and merge.
8. ???
9. Profit

## Helpful Tips

### Other remotes

You can add "remotes" to github that refer to other people's robot code repos. This allows you to, for example, take a look at someone else's code to look over it, you would be able to `git checkout wesley/branch-that-breaks-everything` to see it. To add a remote, just do `git remote add <name_of_person> https://github.com/<username>/robot-code`. Once you've done this, you can use `git fetch <name_of_person>` to get updated code from other people's repos!

### Setting up a new computer

Although we have a third_party directory for the majority of our dependencies, our code still requires some external ones. These are (for manual installation):
  * `opencv2` (apt package: `libopencv-dev`; aur package `opencv2-git`)
  * `clang` (apt/aor package:`clang`)
  * `clang-format` (apt/aor package: `clang-format`)
  * `python2` (apt package: `python`; aor package: `python2`)
  * `python2-pip` (optional; apt package: `python-pip`; aor package `python2-pip`) Installing from both aor/apt/aur and pip *WILL* break python.
  * Python Deps:
    * `numpy` (apt package: `python-numpy`; aor package: `python2-numpy`; pip package: `numpy`)
    * `scipy` (apt package: `python-scipy`; aor package: `python2-scipy`; pip package: `scipy`)
    * `matplotlib` (apt package: `python-matplotlib`; aor package: `python2-matplotlib`; pip package `matplotlib`)
    * `tk` (apt package: `python-tk`; aor package: `tk`; pip package: none, apt/aor/aur/source required for install)
    * `gflags` (apt package: `python-gflags`; aor package: `python2-gflags`; pip package: `gflags`)
    * `glog` (apt/aor/aur package: none (compile from source required); pip package: `glog`)
  * WPILib toolchain (apt ppa: `ppa:wpilib/toolchain`, apt package: `frc-toolchain`, aur package: `frc-2017`)
  * Bazel: (apt users: follow instructions at https://bazel.build/versions/master/docs/install.html#ubuntu; aur: `bazel`)

If you're running Arch/Manjaro or Ubuntu/Debian (_should work with debian_), then simply clone this repository and run:
`./scripts/setup/frcsetup-arch` or `./scripts/setup/frcsetup-ubuntu`
