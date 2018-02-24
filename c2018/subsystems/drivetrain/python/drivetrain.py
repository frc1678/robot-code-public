#!/usr/bin/python

from third_party.frc971.control_loops.python import drivetrain
from third_party.frc971.control_loops.python import control_loop
import sys

import gflags
import glog

FLAGS = gflags.FLAGS

gflags.DEFINE_bool('plot', False, 'If true, plot the loop response.')

mass = 55.0
distribution_radius = 0.4

kDrivetrain = drivetrain.DrivetrainParams(J = mass * distribution_radius ** 2.0,
                                          mass = mass,
                                          robot_radius = 0.426,
                                          wheel_radius = 6.0 * 0.0254 / 2.0,
                                          G_high = (12.0 / 50.0) * (18.0 / 46.0) * (50.0 / 34.0),
                                          G_low = (12.0 / 50.0) * (18.0 / 46.0) * (34.0 / 50.0),
                                          motor_type = control_loop.MiniCIM(),
                                          num_motors = 3,
                                          q_pos_low = 0.22,
                                          q_pos_high = 0.24,
                                          q_vel_low = 2.0,
                                          q_vel_high = 2.0,
                                          )

def main(argv):
  argv = FLAGS(argv)
  glog.init()

  if FLAGS.plot:
    drivetrain.PlotDrivetrainMotions(kDrivetrain)
  elif len(argv) != 5:
    print("Expected .h file name and .cc file name")
  else:
    # Write the generated constants out to a file.
    drivetrain.WriteDrivetrain(argv[1:3], argv[3:5], 'c2018', kDrivetrain)

if __name__ == '__main__':
  sys.exit(main(sys.argv))
