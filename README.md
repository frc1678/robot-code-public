# 1678 Robot Code
[![Build Status](https://travis-ci.com/frc1678/robot-code.svg?token=zuwbfFQzhVsdFka5YhYy&branch=master)](https://travis-ci.com/frc1678/robot-code)

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
bazel test //c2017/submarine_subsystem/...
```

## Contributing

Here's how to get your code into the main robot repository:

###If you've just joined the team:
1. Make an account on [GitHub](https://github.com/).
2. Ask the robot programming lead to add your account to the frc1678 robot programming team.

###If it's the first time you've contributed to this repo:
1. Fork this repo
  1. Log onto github and navigate to the repo [robot-code](https://github.com/frc1678/robot-code).
  2. Click the "Fork" button in the upper right corner of the screen.
2. Clone your forked repo.
  * `git clone https://github.com/<your_name>/robot-code.git`, where <your_name> is your github username.

###Any time you want to make a change:
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
  * At least 2 people (at least one mentor) must review (and approve of) your changes before they are merged.
  * If there are any concerns about your pull request, fix them. Depending on how severe the concerns are, the pull request may be merged without it, but everyone will be happier if you fix your code. To update your PR, just push to the branch on your forked repo.
7. ???
8. Profit

## Helpful Tips

### Other remotes

You can add "remotes" to github that refer to other people's robot code repos. This allows you to, for example, take a look at someone else's code to look over it, you would be able to `git checkout wesley/branch-that-breaks-everything` to see it. To add a remote, just do `git remote add https://github.com/<username>/robot-code`. Once you've done this, you can use `git fetch <remote_name>` to get updated code from other people's repos!
