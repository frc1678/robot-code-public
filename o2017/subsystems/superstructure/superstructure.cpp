#include "o2017/subsystems/superstructure/superstructure.h"
#include "o2017/queue_manager/queue_manager.h"

namespace o2017 {

namespace superstructure {

Superstructure::Superstructure() {}

void Superstructure::Update() {
  auto maybe_goal = QueueManager::GetInstance()->superstructure_goal_queue()->ReadLastMessage();
  auto maybe_input = QueueManager::GetInstance()->superstructure_input_queue()->ReadLastMessage();
  if (maybe_goal && maybe_input) {
    auto goal = *maybe_goal;
    auto input = *maybe_input;

    SuperstructureOutputProto output;
    SuperstructureStatusProto status;

    climber_.Update(input, goal, &output, &status, true);

    output->set_hp_gear_extend(goal->hp_gear());

    QueueManager::GetInstance()->superstructure_output_queue()->WriteMessage(output);
    QueueManager::GetInstance()->superstructure_status_queue()->WriteMessage(status);
  }
}

}  // namespace superstructure

}  // namespace o2017
