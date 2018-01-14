#include "c2018/subsystems/climber/climber.h"

namespace c2018 {

namespace climber {

Climber::Climber() : goal_reader_ { QueueManager<ClimbGoal>::Fetch()->MakeReader(); }
{}

void Climber::Update() {
  ClimbGoal goal;
  if (goal_reader_.ReadLastMessage(&goal)) {
    // use goal here
  }
}

}  // namespace climber

}  // namespace c2018