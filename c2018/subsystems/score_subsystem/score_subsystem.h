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

constexpr double kElevatorBottom = 0;
constexpr double kElevatorFirstStage = 1;
constexpr double kElevatorSecondStage = 2;
constexpr double kElevatorFirstCube = 0;
constexpr double kElevatorSecondCube = 0.3;
constexpr double kElevatorThirdCube = 0.6;
constexpr double kElevatorScoreLow = 0.6;
constexpr double kElevatorScoreHigh = 2;

class ScoreSubsystem {
 public:
  ScoreSubsystem();
  void Update();

 private:
  c2018::score_subsystem::elevator::ElevatorController elevator_;
  c2018::score_subsystem::wrist::WristController wrist_;

  c2018::score_subsystem::ScoreSubsystemGoalQueue::QueueReader goal_reader_;
  c2018::score_subsystem::ScoreSubsystemStatusQueue* status_queue_;
  c2018::score_subsystem::ScoreSubsystemInputQueue::QueueReader input_reader_;
  c2018::score_subsystem::ScoreSubsystemOutputQueue* output_queue_;
  muan::wpilib::DriverStationQueue::QueueReader ds_status_reader_;
  double elevator_height;

  double wrist_angle;
  c2018::score_subsystem::IntakeMode intake_mode;
  c2018::score_subsystem::ScoreGoal score_mode;

};

}  // namespace score_subsystem
}  // namespace c2018

#endif  // C2018_SUBSYSTEMS_SCORE_SUBSYSTEM_SCORE_SUBSYSTEM_H_
