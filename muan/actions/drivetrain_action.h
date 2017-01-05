#ifndef MUAN_ACTIONS_DRIVETRAIN_ACTION_H_
#define MUAN_ACTIONS_DRIVETRAIN_ACTION_H_

#include "third_party/frc971/control_loops/drivetrain/queue_types.h"

namespace muan {

namespace actions {

struct DrivetrainProperties {
  double max_angular_velocity, max_angular_acceleration;
  double max_forward_velocity, max_forward_acceleration;
  double wheelbase_radius;
};

class DrivetrainAction {
 public:
  // Update the action, sending a new message to the queue and returning true if
  // the action is still running.
  virtual bool Update();

  static DrivetrainAction DriveStraight(
      double distance, DrivetrainProperties properties,
      frc971::control_loops::drivetrain::GoalQueue* goal_queue,
      frc971::control_loops::drivetrain::StatusQueue* status_queue,
      double termination_distance = 2e-2, double termination_velocity = 1e-2);

  static DrivetrainAction PointTurn(
      double angle, DrivetrainProperties properties,
      frc971::control_loops::drivetrain::GoalQueue* goal_queue,
      frc971::control_loops::drivetrain::StatusQueue* status_queue,
      double termination_distance = 2e-2, double termination_velocity = 1e-2);

  static DrivetrainAction SwoopTurn(
      double distance, double angle, DrivetrainProperties properties,
      frc971::control_loops::drivetrain::GoalQueue* goal_queue,
      frc971::control_loops::drivetrain::StatusQueue* status_queue,
      double termination_distance = 2e-2, double termination_velocity = 1e-2);

 protected:
  DrivetrainAction(DrivetrainProperties properties, double gl, double gr,
                   double gvl, double gvr, double td, double tv,
                   frc971::control_loops::drivetrain::GoalQueue* gq,
                   frc971::control_loops::drivetrain::StatusQueue* sq)

      void SendMessage();
  bool IsTerminated() const;

  DrivetrainProperties properties_;

  double goal_left_, goal_velocity_left_, goal_right_, goal_velocity_right_;
  double threshold_distance_, threshold_velocity_;

  frc971::control_loops::drivetrain::GoalQueue* goal_queue_;
  frc971::control_loops::drivetrain::StatusQueue* status_queue_;
};

class DriveSCurveAction : public DrivetrainAction {
 public:
  DriveSCurveAction(double distance, double angle,
                    DrivetrainProperties properties,
                    frc971::control_loops::drivetrain::GoalQueue* gq,
                    frc971::control_loops::drivetrain::StatusQueue* sq,
                    double termination_distance = 2e-2,
                    double termination_velocity = 1e-2);
  bool Update() override;

 private:
  bool FinishedFirst();

  bool finished_first_{false};
  double end_left_, end_right_;

  DrivetrainProperties properties_;
};

}  // actions

}  // muan

#endif  // MUAN_ACTIONS_DRIVETRAIN_ACTION_H_
