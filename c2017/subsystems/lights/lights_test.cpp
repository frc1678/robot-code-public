#include "gtest/gtest.h"
#include "c2017/subsystems/lights/lights.h"
#include "c2017/queue_manager/queue_manager.h"

//not doing test fixture because the time it would take to make it work outweighs the time it takes to do everything else 

TEST(LightColors, NoVisionSignal) {
  c2017::vision::VisionStatusProto vision_status;
  vision_status->set_has_connection(false);
  c2017::QueueManager::GetInstance().vision_status_queue().WriteMessage(vision_status);
  c2017::lights::Lights lights;
  aos::time::EnableMockTime(aos::monotonic_clock::epoch());
  lights.Update();
  auto lights_reading = c2017::QueueManager::GetInstance().lights_output_queue().ReadLastMessage();
  if (lights_reading) {
    EXPECT_FALSE(lights_reading.value()->red());
    EXPECT_FALSE(lights_reading.value()->green());
    EXPECT_FALSE(lights_reading.value()->blue()); //creates white
  }
  aos::time::IncrementMockTime(std::chrono::milliseconds(250));
  lights.Update();
  lights_reading = c2017::QueueManager::GetInstance().lights_output_queue().ReadLastMessage();
  if (lights_reading) {
    EXPECT_TRUE(lights_reading.value()->red());
    EXPECT_TRUE(lights_reading.value()->green());
    EXPECT_TRUE(lights_reading.value()->blue()); //is off
  }
}

TEST(LightColors, NotCalibrated) {
    muan::wpilib::gyro::GyroQueue gyro_queue;
    gyro_queue->set_calibration_time_left(5);
}
