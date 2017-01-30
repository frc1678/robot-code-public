#include "gtest/gtest.h"
#include "c2017/subsystems/lights/lights.h"
#include "c2017/queue_manager/queue_manager.h"

// not doing test fixture because the time it would take to make it work
// outweighs the time it takes to do everything else

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
    EXPECT_FALSE(lights_reading.value()->blue());  // is off
  }
  aos::time::IncrementMockTime(std::chrono::milliseconds(250));
  lights.Update();
  lights_reading = c2017::QueueManager::GetInstance().lights_output_queue().ReadLastMessage();
  if (lights_reading) {
    EXPECT_TRUE(lights_reading.value()->red());
    EXPECT_TRUE(lights_reading.value()->green());
    EXPECT_TRUE(lights_reading.value()->blue());  // creates white
  }
}

TEST(LightColors, NotCalibrated) {
  muan::wpilib::gyro::GyroMessageProto gyro_queue;

  gyro_queue->set_calibration_time_left(5);

  c2017::QueueManager::GetInstance().gyro_queue()->WriteMessage(gyro_queue);

  c2017::lights::Lights lights;
  lights.Update();
  auto lights_reading = c2017::QueueManager::GetInstance().lights_output_queue().ReadLastMessage();
  if (lights_reading) {
    EXPECT_FALSE(lights_reading.value()->red());
    EXPECT_FALSE(lights_reading.value()->green());
    EXPECT_TRUE(lights_reading.value()->blue());  // creates blue
  }
}

TEST(LightColors, HpLoadBalls) {
  c2017::vision::VisionStatusProto vision_status;
  muan::wpilib::gyro::GyroMessageProto gyro_queue;
  c2017::intake_group::IntakeGroupGoalProto intake_group_goal_queue;

  vision_status->set_has_connection(true);
  gyro_queue->set_calibration_time_left(0);
  intake_group_goal_queue->set_hp_load_type(c2017::intake_group::HpLoadType::HP_LOAD_BALLS);

  c2017::QueueManager::GetInstance().intake_group_goal_queue().WriteMessage(intake_group_goal_queue);
  c2017::QueueManager::GetInstance().gyro_queue()->WriteMessage(gyro_queue);
  c2017::lights::Lights lights;

  lights.Update();
  auto lights_reading = c2017::QueueManager::GetInstance().lights_output_queue().ReadLastMessage();
  if (lights_reading) {
    EXPECT_TRUE(lights_reading.value()->red());
    EXPECT_TRUE(lights_reading.value()->green());
    EXPECT_FALSE(lights_reading.value()->blue());  // creates yellow
  }
}

TEST(LightColors, HpLoadGear) {
  c2017::vision::VisionStatusProto vision_status;
  vision_status->set_has_connection(true);
  muan::wpilib::gyro::GyroMessageProto gyro_queue;
  gyro_queue->set_calibration_time_left(0);
  c2017::intake_group::IntakeGroupGoalProto intake_group_goal_queue;
  intake_group_goal_queue->set_hp_load_type(c2017::intake_group::HpLoadType::HP_LOAD_GEAR);
  c2017::QueueManager::GetInstance().intake_group_goal_queue().WriteMessage(intake_group_goal_queue);
  c2017::QueueManager::GetInstance().gyro_queue()->WriteMessage(gyro_queue);
  c2017::lights::Lights lights;
  lights.Update();
  auto lights_reading = c2017::QueueManager::GetInstance().lights_output_queue().ReadLastMessage();
  if (lights_reading) {
    EXPECT_TRUE(lights_reading.value()->red());
    EXPECT_FALSE(lights_reading.value()->green());
    EXPECT_TRUE(lights_reading.value()->blue());  // creates pink
  }
}

TEST(LightColors, HpLoadBoth) {
  c2017::vision::VisionStatusProto vision_status;
  muan::wpilib::gyro::GyroMessageProto gyro_queue;
  c2017::intake_group::IntakeGroupGoalProto intake_group_goal_queue;

  vision_status->set_has_connection(true);

  gyro_queue->set_calibration_time_left(0);

  intake_group_goal_queue->set_hp_load_type(c2017::intake_group::HpLoadType::HP_LOAD_BOTH);

  c2017::QueueManager::GetInstance().vision_status_queue().WriteMessage(vision_status);
  c2017::QueueManager::GetInstance().intake_group_goal_queue().WriteMessage(intake_group_goal_queue);
  c2017::QueueManager::GetInstance().gyro_queue()->WriteMessage(gyro_queue);

  c2017::lights::Lights lights;
  aos::time::EnableMockTime(aos::monotonic_clock::epoch());
  lights.Update();
  auto lights_reading = c2017::QueueManager::GetInstance().lights_output_queue().ReadLastMessage();
  if (lights_reading) {
    EXPECT_TRUE(lights_reading.value()->red());
    EXPECT_FALSE(lights_reading.value()->green());
    EXPECT_TRUE(lights_reading.value()->blue());  // creates pink
  }
  aos::time::IncrementMockTime(std::chrono::milliseconds(1000));
  lights.Update();
  lights_reading = c2017::QueueManager::GetInstance().lights_output_queue().ReadLastMessage();
  if (lights_reading) {
    EXPECT_TRUE(lights_reading.value()->red());
    EXPECT_TRUE(lights_reading.value()->green());
    EXPECT_FALSE(lights_reading.value()->blue());  // creates yellow
  }
}

TEST(LightColors, VisionNotAlligned) {
  c2017::vision::VisionStatusProto vision_status;
  muan::wpilib::gyro::GyroMessageProto gyro_queue;
  c2017::intake_group::IntakeGroupGoalProto intake_group_goal_queue;

  vision_status->set_target_found(true);
  vision_status->set_has_connection(true);
  vision_status->set_aligned(false);

  gyro_queue->set_calibration_time_left(0);

  intake_group_goal_queue->set_hp_load_type(c2017::intake_group::HpLoadType::HP_LOAD_NONE);

  c2017::QueueManager::GetInstance().vision_status_queue().WriteMessage(vision_status);
  c2017::QueueManager::GetInstance().intake_group_goal_queue().WriteMessage(intake_group_goal_queue);
  c2017::QueueManager::GetInstance().gyro_queue()->WriteMessage(gyro_queue);

  c2017::lights::Lights lights;
  lights.Update();
  auto lights_reading = c2017::QueueManager::GetInstance().lights_output_queue().ReadLastMessage();
  if (lights_reading) {
    EXPECT_TRUE(lights_reading.value()->red());
    EXPECT_TRUE(lights_reading.value()->green());
    EXPECT_FALSE(lights_reading.value()->blue());  // creates yellow
  }
}

TEST(LightColors, VisionTargetNotFound) {
  c2017::vision::VisionStatusProto vision_status;
  muan::wpilib::gyro::GyroMessageProto gyro_queue;
  c2017::intake_group::IntakeGroupGoalProto intake_group_goal_queue;

  vision_status->set_target_found(false);
  vision_status->set_has_connection(true);
  vision_status->set_aligned(false);

  gyro_queue->set_calibration_time_left(0);

  intake_group_goal_queue->set_hp_load_type(c2017::intake_group::HpLoadType::HP_LOAD_NONE);

  c2017::QueueManager::GetInstance().vision_status_queue().WriteMessage(vision_status);
  c2017::QueueManager::GetInstance().intake_group_goal_queue().WriteMessage(intake_group_goal_queue);
  c2017::QueueManager::GetInstance().gyro_queue()->WriteMessage(gyro_queue);

  c2017::lights::Lights lights;
  lights.Update();
  auto lights_reading = c2017::QueueManager::GetInstance().lights_output_queue().ReadLastMessage();
  if (lights_reading) {
    EXPECT_TRUE(lights_reading.value()->red());
    EXPECT_FALSE(lights_reading.value()->green());
    EXPECT_FALSE(lights_reading.value()->blue());  // creates red
  }
}

TEST(LightColors, VisionAligned) {
  c2017::vision::VisionStatusProto vision_status;
  muan::wpilib::gyro::GyroMessageProto gyro_queue;
  c2017::intake_group::IntakeGroupGoalProto intake_group_goal_queue;

  vision_status->set_target_found(true);
  vision_status->set_has_connection(true);
  vision_status->set_aligned(true);

  gyro_queue->set_calibration_time_left(0);

  intake_group_goal_queue->set_hp_load_type(c2017::intake_group::HpLoadType::HP_LOAD_NONE);

  c2017::QueueManager::GetInstance().vision_status_queue().WriteMessage(vision_status);
  c2017::QueueManager::GetInstance().intake_group_goal_queue().WriteMessage(intake_group_goal_queue);
  c2017::QueueManager::GetInstance().gyro_queue()->WriteMessage(gyro_queue);

  c2017::lights::Lights lights;
  lights.Update();
  auto lights_reading = c2017::QueueManager::GetInstance().lights_output_queue().ReadLastMessage();
  if (lights_reading) {
    EXPECT_FALSE(lights_reading.value()->red());
    EXPECT_TRUE(lights_reading.value()->green());
    EXPECT_FALSE(lights_reading.value()->blue());  // creates green
  }
}
