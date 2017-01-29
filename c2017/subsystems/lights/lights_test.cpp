#include "gtest/gtest.h"
#include "c2017/subsystems/lights/lights.h"
#include "c2017/queue_manager/queue_manager.h"

TEST(asdasd) {
  IntakeGroupProto intake_goals;
  intake_goals->setasdasd(adasd);

  QueueManager::GetInstance().intake_grouptadasdasdasd().WriteMessage(intake_goals);

  lights.Updage();
  
  lights_vals = QueueManager::geedasdasD.light_output_queue().ReadLastMessage();
  if lights_vbals {
      light_balsada.red();

  
  } else {
    FAIL();
  }
}
//not doing test fixture because the time it would take to make it work outweighs the time it takes to do everything else 

TEST(LightColors, NoVisionSignal) {
  IntakeGroupGoalProto intake_group_goals;
  intake_group_goals->set_ground_intake_position(
}


