import sys
import subprocess as sp
import re
import os
from distutils import spawn
import argparse

"""
Deploys a program and all related files needed for execution. This should usually not be run directly - use
the muan_deploy bazel rule from deploy.bzl, which runs this script.
"""

def get_args():
    parser = argparse.ArgumentParser(description = "Deploy a robot program and all related files needed for execution")
    parser.add_argument('--main', dest='main_binary', help="The main binary to run with robotCommand", required=True)
    parser.add_argument('--default-target', dest='default_target', help="The default target to deploy to. Should only be set by the muan_deploy bazel rule.", required=True)
    parser.add_argument('--user', dest='user', help="The user to log in and deploy as", default='admin')
    parser.add_argument('--target', dest='target', help="The address of the RoboRIO to deploy to", default=None)
    parser.add_argument('--team', dest='team', help="The team number to deploy to. Setting this option will override --target as roborio-####-frc.local.", default=None)
    parser.add_argument('--port', dest='port', default='22')
    parser.add_argument('--deploy-path', dest='deploy_path', help="The path on the RoboRIO to deploy code to", default='/home/lvuser/robot_code')
    parser.add_argument('--robot-command', dest='command', help="The robotCommand file to create", default='/home/lvuser/start_robot_code')
    parser.add_argument('--password', dest='password', help="The password for the user on the RoboRIO")
    args = parser.parse_args()
    if args.target is None:
        if args.team is not None:
            args.target = 'roborio-{}-frc.local'.format(args.team)
        else:
            args.target = args.default_target

    return args

# Shamelessly stolen from 971's downloader script :)
def install(ssh_target, pkg):
    """Installs a package from NI on the ssh target."""
    print('Installing', pkg)
    PKG_URL = 'http://download.ni.com/ni-linux-rt/feeds/2015/arm/ipk/cortexa9-vfpv3/' + pkg
    sp.check_call(['wget', PKG_URL, '-O', pkg])
    try:
        sp.check_call(['scp', pkg, ssh_target + ':/tmp/' + pkg])
        sp.check_call(['ssh', ssh_target, 'opkg', 'install', '/tmp/' + pkg])
        sp.check_call(['ssh', ssh_target, 'rm', '/tmp/' + pkg])
    finally:
        sp.check_call(['rm', pkg])

def get_ssh_command(port):
    """
    Gets an ssh command, using prefixed with sshpass if it is installed and specifying a port. This command
    can be used to call ssh directly or with some other utility like rsync.
    """
    sshpass_path = spawn.find_executable('sshpass')
    return ([sshpass_path, '-e'] if sshpass_path is not None else []) + ['ssh', '-p', '%s' % port]

def main():
    files = []
    opts = []

    options = get_args()

    if options.password is None:
        options.password = ""

    # Set the password to be used by sshpass
    os.environ["SSHPASS"] = options.password

    # Where we actually want to ssh into
    ssh_target = '{}@{}'.format(options.user, options.target)

    # An ssh target combined with the path to deploy to
    assert os.path.isabs(options.deploy_path), "Path must be absolute: {}".format(options.deploy_path)
    ssh_deploy_path = "{}:{}".format(ssh_target, options.deploy_path)

    # The ssh command to use
    ssh_command = get_ssh_command(options.port)

    # rsync all of the runfiles over. This will also copy this script over, which isn't the cleanest way to
    # do it, but it works and this file is small enough that it doesn't matter. Also, if this file doesn't change (which it shouldn't) there will be no problem.
    rsync = ['rsync', '-e', ' '.join(ssh_command), '-c', '-v', '-z', '-r', '-L', '.', ssh_deploy_path]

    # This will look like (cd /home/lvuser/robot_code && ./c2017/frc1678).
    robot_command_contents = '''cd {};./muan/autostart/autostart /tmp/autostart.pid {}'''.format(options.deploy_path, options.main_binary)

    # The ssh command that we're going to run to create robotCommand
    ssh = ssh_command + [
           ssh_target,
           'echo "{robot_command_contents}" > {robot_command_path} && chmod +x {robot_command_path}'.format(
               robot_command_contents = robot_command_contents,
               robot_command_path = options.command)
    ]

    # The command to set up running the autostart script on startup
    # TODO(Wesley) Make this only run when needed. Also, condense everything
    # into one ssh session.
    ssh_autostart_command = ssh_command + [
        ssh_target,
        'echo "{} &" > /etc/init.d/start_robot_code && \
         chmod +x /etc/init.d/start_robot_code && \
         update-rc.d start_robot_code defaults 100'.format(options.command)
    ]

    # The ssh command that's used to set the SUID bit on the robot code
    ssh_suid_command = ssh_command + [
        ssh_target,
        "chmod +s {}".format(os.path.join(options.deploy_path, options.main_binary))
    ]

    # The command to kill the currently running robot code. This will cause it
    # to be restarted by the autostart script, *if* the autostart script is
    # running. This means that the first time that this script is run on a
    # newly-flashed roborio, the code will not get automatically started.
    ssh_kill_code_command = ssh_command + [
        ssh_target,
        "kill $(pidof {})".format(options.main_binary)
    ]

    # Try to rsync the files over and run an ssh command to create the robotCommand.
    try:
        print("Running rsync command: {}".format(' '.join(rsync)))
        sp.check_call(rsync)
    except sp.CalledProcessError as e:
        # If it doesn't work, try installing rsync on the RoboRIO.
        if e.returncode == 127:
            print('Unconfigured roboRIO, installing rsync.')
            install(ssh_target, 'libattr1_2.4.47-r0.36_cortexa9-vfpv3.ipk')
            install(ssh_target, 'libacl1_2.2.52-r0.36_cortexa9-vfpv3.ipk')
            install(ssh_target, 'rsync_3.1.0-r0.7_cortexa9-vfpv3.ipk')
            sp.check_call(rsync)
        elif e.returncode == 12:
            print('Incorrect password')
            return e.returncode
        elif e.returncode == 255:
            print("SSH error - check that the address is correct")
            return e.returncode
        else:
            raise e
    print("Running ssh command: {}".format(' '.join(ssh)))
    sp.check_call(ssh)
    print("Running set SUID command: {}".format(' '.join(ssh_suid_command)))
    sp.check_call(ssh_suid_command)
    print("Running set autostart setup command: {}".format(' '.join(ssh_autostart_command)))
    sp.check_call(ssh_autostart_command)
    print("Running kill robot code command: {}".format(' '.join(ssh_kill_code_command)))
    sp.check_call(ssh_kill_code_command)
    print("Deploying completed successfully.")

if __name__ == '__main__':
    exit(main())
