#ifndef MUAN_ACTIONS_DRIVETRAIN_ACTION_H_
#define MUAN_ACTIONS_DRIVETRAIN_ACTION_H_

#include "third_party/frc971/control_loops/drivetrain/queue_types.h"

namespace muan {
namespace actions {

// Drivetrain properties we use in actions
struct DrivetrainProperties {
  double max_angular_velocity{0.0}, max_angular_acceleration{0.0};
  double max_forward_velocity{0.0}, max_forward_acceleration{0.0};
  double wheelbase_radius{0.0};

  DrivetrainProperties(double max_angular_velocity,
                       double max_angular_acceleration,
                       double max_forward_velocity,
                       double max_forward_acceleration,
                       double wheelbase_radius);
};

struct DrivetrainTermination {
  double forward{0.0}, forward_velocity{0.0};
  double angular{0.0}, angular_velocity{0.0};

  DrivetrainTermination(double forward, double forward_velocity, double angular,
                        double angular_velocity);
};

struct DrivetrainActionParams {
  // The desired arc length to travel (in meters)
  double desired_forward_distance{0.0};
  // The desired angular displacement (in radians, counterclockwise is positive)
  double desired_angular_displacement{0.0};

  // The requested gear for the robot
  bool high_gear{false};

  // True means we end at a nonzero forward velocity
  bool follow_through{false};

  // True means we rely on the sensors for termination, false means we just do
  // it based on when the profile
  // completes.
  bool closed_loop_termination{true};

  // True means to avoid doing any calculations on the constraints and to just
  // tell the robot to use the
  // constraints specified in the properties.
  bool literal_constraints{false};

  // Whether or not to turn to an absolute angle or a relative value
  bool angle_is_absolute{false};

  // The termination condition. Ignored if closed_loop_termination is false.
  DrivetrainTermination termination{0.02, 0.02, 0.04, 0.02};
};

class DrivetrainAction {
 public:
  DrivetrainAction(
      DrivetrainProperties properties,
      frc971::control_loops::drivetrain::GoalQueue* goal_queue,
      frc971::control_loops::drivetrain::StatusQueue* status_queue);
  virtual ~DrivetrainAction() = default;

  // Execute the specified profile
  void ExecuteDrive(DrivetrainActionParams params);

  // Update the action, sending a new message to the queue and returning true if
  // the action is still running.
  virtual bool Update();

 protected:
  // Send a goal message into the queue
  void SendMessage();

  // Is the action done yet?
  bool IsTerminated();

  const DrivetrainProperties properties_;
  DrivetrainActionParams current_params_;

  DrivetrainTermination termination_{0.02, 0.02, 0.04, 0.02};

  double goal_left_, goal_velocity_left_, goal_right_, goal_velocity_right_;

  // Motion profile properties
  double max_forward_velocity_, max_angular_velocity_,
      max_forward_acceleration_, max_angular_acceleration_;
  bool high_gear_;

  bool closed_loop_termination_;

  // Have the left and right goals finished? Used for open-loop termination.
  bool left_complete_{false}, right_complete_{false};
  double last_left_, last_right_;

  frc971::control_loops::drivetrain::GoalQueue* const goal_queue_;
  const frc971::control_loops::drivetrain::StatusQueue* const status_queue_;
};

}  // namespace actions
}  // namespace muan

#endif  // MUAN_ACTIONS_DRIVETRAIN_ACTION_H_
