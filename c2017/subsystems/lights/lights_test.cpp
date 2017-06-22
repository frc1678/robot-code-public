#include "gtest/gtest.h"
#include "c2017/subsystems/lights/lights.h"
#include "c2017/queue_manager/queue_manager.h"

TEST(LightColors, NoVisionSignal) {
  c2017::QueueManager::GetInstance()->Reset();

  c2017::vision::VisionStatusProto vision_status;
  muan::wpilib::DriverStationProto driver_station_proto;

  vision_status->set_has_connection(false);
  driver_station_proto->set_mode(RobotMode::TELEOP);

  c2017::QueueManager::GetInstance()->vision_status_queue()->WriteMessage(vision_status);
  c2017::QueueManager::GetInstance()->driver_station_queue()->WriteMessage(driver_station_proto);

  c2017::lights::Lights lights;
  aos::time::EnableMockTime(aos::monotonic_clock::epoch());
  lights.Update();
  auto lights_reading = c2017::QueueManager::GetInstance()->lights_output_queue()->ReadLastMessage();
  if (lights_reading) {
    EXPECT_FALSE(lights_reading.value()->red());
    EXPECT_FALSE(lights_reading.value()->green());
    EXPECT_FALSE(lights_reading.value()->blue());  // is off
  } else {
    FAIL();
  }

  aos::time::IncrementMockTime(std::chrono::milliseconds(250));
  lights.Update();
  lights_reading = c2017::QueueManager::GetInstance()->lights_output_queue()->ReadLastMessage();
  if (lights_reading) {
    EXPECT_FALSE(lights_reading.value()->red());
    EXPECT_TRUE(lights_reading.value()->green());
    EXPECT_TRUE(lights_reading.value()->blue());  // creates teal
  } else {
    FAIL();
  }
}

TEST(LightColors, NotCalibrated) {
  c2017::QueueManager::GetInstance()->Reset();

  muan::wpilib::gyro::GyroMessageProto gyro_proto;

  gyro_proto->set_calibration_time_left(5);

  c2017::QueueManager::GetInstance()->gyro_queue()->WriteMessage(gyro_proto);

  c2017::lights::Lights lights;
  lights.Update();
  auto lights_reading = c2017::QueueManager::GetInstance()->lights_output_queue()->ReadLastMessage();
  if (lights_reading) {
    EXPECT_TRUE(lights_reading.value()->red());
    EXPECT_FALSE(lights_reading.value()->green());
    EXPECT_FALSE(lights_reading.value()->blue());  // creates red
  } else {
    FAIL();
  }
}

TEST(LightColors, VisionNotAlligned) {
  c2017::QueueManager::GetInstance()->Reset();

  c2017::vision::VisionStatusProto vision_status;
  muan::wpilib::gyro::GyroMessageProto gyro_proto;
  c2017::intake_group::IntakeGroupGoalProto intake_group_goal_proto;
  muan::wpilib::DriverStationProto driver_station_proto;

  vision_status->set_target_found(true);
  vision_status->set_has_connection(true);
  vision_status->set_aligned(false);

  gyro_proto->set_calibration_time_left(0);

  driver_station_proto->set_mode(RobotMode::TELEOP);

  c2017::QueueManager::GetInstance()->vision_status_queue()->WriteMessage(vision_status);
  c2017::QueueManager::GetInstance()->intake_group_goal_queue()->WriteMessage(intake_group_goal_proto);
  c2017::QueueManager::GetInstance()->gyro_queue()->WriteMessage(gyro_proto);
  c2017::QueueManager::GetInstance()->driver_station_queue()->WriteMessage(driver_station_proto);

  c2017::lights::Lights lights;
  lights.Update();
  auto lights_reading = c2017::QueueManager::GetInstance()->lights_output_queue()->ReadLastMessage();
  if (lights_reading) {
    EXPECT_TRUE(lights_reading.value()->red());
    EXPECT_TRUE(lights_reading.value()->green());
    EXPECT_FALSE(lights_reading.value()->blue());  // creates yellow
  } else {
    FAIL();
  }
}

TEST(LightColors, VisionTargetNotFound) {
  c2017::QueueManager::GetInstance()->Reset();

  c2017::vision::VisionStatusProto vision_status;
  muan::wpilib::gyro::GyroMessageProto gyro_proto;
  c2017::intake_group::IntakeGroupGoalProto intake_group_goal_proto;
  muan::wpilib::DriverStationProto driver_station_proto;

  vision_status->set_target_found(false);
  vision_status->set_has_connection(true);
  vision_status->set_aligned(false);

  gyro_proto->set_calibration_time_left(0);

  driver_station_proto->set_mode(RobotMode::TELEOP);

  c2017::QueueManager::GetInstance()->vision_status_queue()->WriteMessage(vision_status);
  c2017::QueueManager::GetInstance()->intake_group_goal_queue()->WriteMessage(intake_group_goal_proto);
  c2017::QueueManager::GetInstance()->gyro_queue()->WriteMessage(gyro_proto);
  c2017::QueueManager::GetInstance()->driver_station_queue()->WriteMessage(driver_station_proto);

  c2017::lights::Lights lights;
  lights.Update();
  auto lights_reading = c2017::QueueManager::GetInstance()->lights_output_queue()->ReadLastMessage();
  if (lights_reading) {
    EXPECT_TRUE(lights_reading.value()->red());
    EXPECT_FALSE(lights_reading.value()->green());
    EXPECT_FALSE(lights_reading.value()->blue());  // creates red
  } else {
    FAIL();
  }
}

TEST(LightColors, VisionAligned) {
  c2017::QueueManager::GetInstance()->Reset();

  c2017::vision::VisionStatusProto vision_status;
  muan::wpilib::gyro::GyroMessageProto gyro_proto;
  c2017::intake_group::IntakeGroupGoalProto intake_group_goal_proto;
  muan::wpilib::DriverStationProto driver_station_proto;

  vision_status->set_target_found(true);
  vision_status->set_has_connection(true);
  vision_status->set_aligned(true);

  gyro_proto->set_calibration_time_left(0);

  driver_station_proto->set_mode(RobotMode::TELEOP);

  c2017::QueueManager::GetInstance()->vision_status_queue()->WriteMessage(vision_status);
  c2017::QueueManager::GetInstance()->intake_group_goal_queue()->WriteMessage(intake_group_goal_proto);
  c2017::QueueManager::GetInstance()->gyro_queue()->WriteMessage(gyro_proto);
  c2017::QueueManager::GetInstance()->driver_station_queue()->WriteMessage(driver_station_proto);

  c2017::lights::Lights lights;
  lights.Update();
  auto lights_reading = c2017::QueueManager::GetInstance()->lights_output_queue()->ReadLastMessage();
  if (lights_reading) {
    EXPECT_FALSE(lights_reading.value()->red());
    EXPECT_TRUE(lights_reading.value()->green());
    EXPECT_FALSE(lights_reading.value()->blue());  // creates green
  } else {
    FAIL();
  }
}

TEST(LightColors, NoDsQueue) {
  c2017::QueueManager::GetInstance()->Reset();

  muan::wpilib::gyro::GyroMessageProto gyro_proto;
  c2017::vision::VisionStatusProto vision_status;

  gyro_proto->set_calibration_time_left(0);
  vision_status->set_has_connection(true);

  c2017::QueueManager::GetInstance()->gyro_queue()->WriteMessage(gyro_proto);
  c2017::QueueManager::GetInstance()->vision_status_queue()->WriteMessage(vision_status);

  c2017::lights::Lights lights;
  lights.Update();
  auto lights_reading = c2017::QueueManager::GetInstance()->lights_output_queue()->ReadLastMessage();
  if (lights_reading) {
    EXPECT_FALSE(lights_reading.value()->red());
    EXPECT_FALSE(lights_reading.value()->green());
    EXPECT_TRUE(lights_reading.value()->blue());  // is blue
  } else {
    FAIL();
  }
}
