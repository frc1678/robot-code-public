#ifndef C2018_SUBSYSTEMS_SCORE_SUBSYSTEM_SCORE_SUBSYSTEM_H_
#define C2018_SUBSYSTEMS_SCORE_SUBSYSTEM_SCORE_SUBSYSTEM_H_

#include <cmath>
#include "c2018/subsystems/score_subsystem/elevator/elevator_controller.h"
#include "c2018/subsystems/score_subsystem/queue_types.h"
#include "c2018/subsystems/score_subsystem/wrist/wrist.h"
#include "muan/queues/queue_manager.h"
#include "muan/wpilib/queue_types.h"

namespace c2018 {
namespace score_subsystem {

static constexpr double kElevatorBottom = 0;
static constexpr double kElevatorFirstStage = 1;
static constexpr double kElevatorSecondStage = 2;
static constexpr double kElevatorFirstCube = 0;
static constexpr double kElevatorSecondCube = 0.3;
static constexpr double kElevatorThirdCube = 0.58;
static constexpr double kElevatorScoreLow = 0.6;
static constexpr double kElevatorScoreMid = 1.5;
static constexpr double kElevatorScoreHigh = 2;
static constexpr double kElevatorStowHeight = 0.0;
static constexpr double kElevatorWristSafeHeight = 0.89;
static constexpr double kElevatorExchangeHeight = 0.05;

static constexpr double kWristForwardAngle = 0 * (M_PI / 180);
static constexpr double kWristStowAngle = 80 * (M_PI / 180);
static constexpr double kWristBackwardAngle = 160 * (M_PI / 180);
static constexpr double kWristSafeAngle = M_PI / 2.0;

class ScoreSubsystem {
 public:
  ScoreSubsystem();
  void Update();

 private:
  void SetGoal(const ScoreSubsystemGoalProto& goal);
  void GoToState(ScoreSubsystemState state);
  void RunStateMachine();

  void BoundGoal(double* elevator_goal, double* wrist_goal) const;

  elevator::ElevatorController elevator_;
  wrist::WristController wrist_;

  ScoreSubsystemGoalQueue::QueueReader goal_reader_;
  ScoreSubsystemInputQueue::QueueReader input_reader_;
  ScoreSubsystemStatusQueue* status_queue_;
  ScoreSubsystemOutputQueue* output_queue_;

  muan::wpilib::DriverStationQueue::QueueReader ds_status_reader_;
  ScoreSubsystemStatusProto status_;

  double elevator_height_;
  double wrist_angle_;

  ScoreSubsystemState state_ = ScoreSubsystemState::CALIBRATING;
};

}  // namespace score_subsystem
}  // namespace c2018

#endif  // C2018_SUBSYSTEMS_SCORE_SUBSYSTEM_SCORE_SUBSYSTEM_H_
