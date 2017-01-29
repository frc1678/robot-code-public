#include "gtest/gtest.h"
#include "c2017/subsystems/lights/lights.h"
#include "c2017/queue_manager/queue_manager.h"

TEST(asdasd) {
  IntakeGroupProto intake_goals;
  intake_goals->setasdasd(adasd);

  QueueManager::GetInstance().intake_grouptadasdasdasd().WriteMessage(intake_goals);

  lights.Update();
  
  lights_vals = QueueManager::geedasdasD.light_output_queue().ReadLastMessage();
  if lights_vbals {
      light_balsada.red();

  
  } else {
    FAIL();
  }
}
//not doing test fixture because the time it would take to make it work outweighs the time it takes to do everything else 

TEST(LightColors, NoVisionSignal) {
  c2017::vision::VisionStatusProto vision_status;
  vision_status = QueueManager::GetInstance().vision_status_queue().WriteMessage
  vision_status->set_has_connection(false);
  Lights lights;
  aos::time::EnableMockTime(aos::monotonic_clock::epoch());
  for (int i = 0; i < 2000; i++) {
    double t = i * 0.005;
    lights.Update();
    auto lights_reading = QueueManager::GetInstance().lights_output_queue().ReadLastMessage();
    if (lights_reading) {
      if (fmod(t, 0.5) < 0.25) {
        EXPECT_TRUE(lights_reading.value()->red());
        EXPECT_TRUE(lights_reading.value()->green());
        EXPECT_TRUE(lights_reading.value()->blue()); //creates white
      } else {
        EXPECT_FALSE(lights_reading.value()->red());
        EXPECT_FALSE(lights_reading.value()->green());
        EXPECT_FALSE(lights_reading.value()->blue()); //is off
      }
    }
    aos::time::IncrementMockTime(std::chrono::milliseconds(5));
  }
}
