import sys
import subprocess as sp
import re
import os
from distutils import spawn

"""
Deploys a program and all related files needed for execution. This should usually not be run directly - use
the muan_deploy bazel rule from deploy.bzl, which runs this script.
"""

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

def main(argv):
    files = []
    opts = []

    options = {}

    # All of the files in the list will come before the --, all other options come after.
    if '--' in argv:
        files = argv[1:argv.index('--')]
        opts = argv[argv.index('--')+1:]
    else:
        files = argv[1:]

    # Each option must be of the form --flag=value (or --flag, which sets the value to true implicitly)
    for opt in opts:
        if opt.startswith('--'):
            if '=' in opt:
                flag_name = opt.split('=')[0][2:]
                flag_value = ''.join(opt.split('=')[1:])
                options[flag_name] = flag_value
            else:
                options[opt] = True

    # The user to log in as
    user = options.get('user', 'admin')

    # The port of the machine to connect to
    port = options.get('port', '22')

    # The address of the machine to deploy to
    target = ''

    if 'target' in options:
        target = options.get('target')
    elif 'team' in options:
        target = 'roborio-{}-frc.local'.format(options.get('team'))
    else:
        target = options.get('default-target')

    assert len(target) > 0, "Can't have an empty target! Use --team=<team number>, --target=<ip>, or --default-target=<ip>"

    # Where to put the actual files on the RoboRIO
    remote_path = options.get('path', '/home/lvuser/robot_code')

    # Where to put the robotCommand file on the RoboRIO
    binary_name = options.get('robot-command', '/home/lvuser/robotCommand')
    binary_path = os.path.dirname(binary_name)

    # Set the password to be used by sshpass
    os.environ["SSHPASS"] = options.get('password', '')

    # The main file that we should run with the robotCommand
    assert 'main' in options, "Must supply the name for a main program file!"
    main = options.get('main')

    all_files = files + [main]

    # An ssh target combined with the path to deploy to
    assert os.path.isabs(remote_path), "Path must be absolute: {}".format(remote_path)
    full_remote_path = "%s@%s:%s" % (user, target, remote_path)

    # The ssh command to use
    ssh_command = get_ssh_command(port)

    # rsync all of the runfiles over. This will also copy this script over, which isn't the cleanest way to
    # do it, but it works and this file is small enough that it doesn't matter. Also, if this file doesn't change (which it shouldn't) there will be no problem.
    rsync = ['rsync', '-e', ' '.join(ssh_command), '-c', '-v', '-z', '-r', '-L', '.', full_remote_path]

    # This will look like (cd /home/lvuser/robot_code && ./c2017/frc1678).
    robot_command_contents = '''(cd {} && ./{})'''.format(remote_path, main)

    # Where we actually want to ssh into
    ssh_target = '%s@%s' % (user, target)

    # The ssh command that we're going to run to create robotCommand
    ssh = ssh_command + [
           ssh_target,
           'echo "{robot_command_contents}" > {robot_command_path} && chmod +x {robot_command_path}'.format(
               robot_command_contents = robot_command_contents,
               robot_command_path = binary_name)
    ]

    # Try to rsync the files over and run an ssh command to create the robotCommand.
    try:
        print("Running rsync command: {}".format(' '.join(rsync)))
        sp.check_call(rsync)
        print("Running ssh command: {}".format(' '.join(ssh)))
        sp.check_call(ssh)
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
        elif e.returncode == 255:
            print("SSH error - check that the address is correct")
        else:
            raise e

    print("Deploying completed successfully.")

if __name__ == '__main__':
    main(sys.argv)
