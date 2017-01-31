#ifndef MUAN_ACTIONS_DRIVETRAIN_ACTION_H_
#define MUAN_ACTIONS_DRIVETRAIN_ACTION_H_

#include "third_party/frc971/control_loops/drivetrain/queue_types.h"

namespace muan {

namespace actions {

// Drivetrain properties we use in actions
struct DrivetrainProperties {
  double max_angular_velocity, max_angular_acceleration;
  double max_forward_velocity, max_forward_acceleration;
  double wheelbase_radius;
};

struct DrivetrainTermination {
  double forward = 0.0, forward_velocity = 0.0;
  double angular = 0.0, angular_velocity = 0.0;
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

  // True means we rely on the sensors for termination, false means we just do it based on when the profile
  // completes.
  bool closed_loop_termination{true};

  // The termination condition. Ignored if closed_loop_termination is false.
  DrivetrainTermination termination{.02, .02, .04, .02};
};

class DrivetrainAction {
 public:
  DrivetrainAction(DrivetrainProperties properties, frc971::control_loops::drivetrain::GoalQueue* goal_queue,
                   frc971::control_loops::drivetrain::StatusQueue* status_queue);

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

  DrivetrainTermination termination_;

  double goal_left_, goal_velocity_left_, goal_right_, goal_velocity_right_;

  // Motion profile properties
  double max_forward_velocity_, max_angular_velocity_, max_forward_acceleration_, max_angular_acceleration_;
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
