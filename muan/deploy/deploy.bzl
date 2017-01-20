def _muan_deploy_impl(ctx):
  ctx.file_action(
    output = ctx.outputs.executable,
    # Run `downloader <files> -- --default-target <default_target> --any-args-passed-to-bazel
    content = '\n'.join([
      '#!/bin/bash',
    ] + [
      'exec %s %s -- --main=%s --default-target=%s "$@"' % (
          ctx.executable._deploy_script.short_path,
          ' '.join([f.short_path for f in ctx.files.data + ctx.files.main]),
          ctx.files.main[0].short_path,
          ctx.attr.default_target)
    ]),
    executable = True,
  )

  return struct(
      runfiles = ctx.runfiles(
        files = ctx.files._deploy_script + ctx.files.data + ctx.files.main,
        collect_data = True,
        collect_default = True,
      ),
      files = set([ctx.outputs.executable])
  )

_muan_deploy = rule(
  implementation = _muan_deploy_impl,
  attrs = {
    '_deploy_script': attr.label(
      executable = True,
      cfg = 'host',
      default = Label('//muan/deploy'),
    ),
    # Any data dependencies of the robot program, including lemonscript libraries.
    'data': attr.label_list(
      mandatory = False,
      allow_files = True,
      cfg = 'data',
    ),
    # The main executable. This should be a robot program.
    'main': attr.label(
      mandatory = True,
      executable = True,
      cfg = 'data',
    ),
    # The default IP address to deploy to. This can be be overridden with --target.
    'default_target': attr.string(
      default = 'roborio-1678-frc.local',
    )
  },
  executable = True,
  outputs = {}
)

"""
A rule to deploy a program and all of its runfiles to the robot.
Usage:
  muan_deploy(
    name = 'deploy',
    main = ':frc1678',
  )

  bazel run --cpu=roborio //c2017:deploy

To pass extra arguments, call bazel like this:
  bazel run --cpu=roborio //c2017:deploy -- <extra arguments>

Arguments:
  --user=<name>   The user to use to log into the RoboRIO. Defaults to admin.

  --target=<ip>   The address of the RoboRIO. Defaults to roborio-1678-frc.local, or the default_target
                  attribute of this rule.
  --team=<team>   A team number to deploy to. This will set the IP address to roborio-<team>-frc.local.

  --path=<path>   The path on the RoboRIO to deploy the binary to. Defaults to /home/lvuser/robot_code.
  --robot-command=<command> The command file to create. /home/lvuser/robotCommand by default

  --port=<port>   The port that the RIO uses for SSH. Defaults to 22.

  --password=<password> The password to use to SSH into the RoboRIO. Defaults to blank.

The deploy script will connect to <user>@{<target>|roborio-<team>-frc.local}:<port> and copy all of the
runfiles into <path> on the RoboRIO. It will then set up a robot command at <command> that calls the binary
specified as main in the rule.
"""
def muan_deploy(main, name, data = []):
  # This is a macro instead of a rule as a workaround for the fact that only deps's and data's runfiles get
  # collected transitively by default. It basically just calls the _muan_deploy rule, which does all of the
  # heavy lifting.
  _muan_deploy(
    main = main,
    name = name,
    data = data + [main],
  )
