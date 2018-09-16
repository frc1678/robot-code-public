#include "muan/control/drivetrain_model.h"
#include <iostream>
#include "gtest/gtest.h"

namespace muan {
namespace control {

TEST(TransmissionModel, Sanity) {
  DriveTransmission::Properties properties;
  properties.motor_kv = properties.motor_kt = 0.0182;
  properties.motor_resistance = 0.09;
  properties.gear_ratio = 0.1;

  double torque_0 = DriveTransmission(properties).CalculateTorque(0.0, 12.0);
  EXPECT_NEAR(DriveTransmission(properties).CalculateTorque(0.0, 6.0),
              0.5 * torque_0, 1e-3);
  EXPECT_LT(DriveTransmission(properties).CalculateTorque(10.0, 6.0), torque_0);

  {
    DriveTransmission::Properties properties_new = properties;
    properties_new.num_motors = 2;
    EXPECT_NEAR(DriveTransmission(properties_new).CalculateTorque(0.0, 12.0),
                2 * torque_0, 1e-3);
  }

  {
    DriveTransmission::Properties properties_new = properties;
    properties_new.efficiency = 0.5;
    EXPECT_NEAR(DriveTransmission(properties_new).CalculateTorque(0.0, 12.0),
                0.5 * torque_0, 1e-3);
  }

  {
    DriveTransmission::Properties properties_new = properties;
    properties_new.gear_ratio = properties.gear_ratio / 2.0;
    EXPECT_NEAR(DriveTransmission(properties_new).CalculateTorque(0.0, 12.0),
                2.0 * torque_0, 1e-3);
  }
}

TEST(TransmissionModel, Inverse) {
  DriveTransmission::Properties properties;
  properties.motor_kv = properties.motor_kt = 0.0182;
  properties.motor_resistance = 0.09;
  properties.gear_ratio = 0.1;

  EXPECT_NEAR(
      DriveTransmission(properties)
          .VoltageFromTorque(
              0.0, DriveTransmission(properties).CalculateTorque(0.0, 12.0)),
      12.0, 1e-3);
  EXPECT_NEAR(
      DriveTransmission(properties)
          .VoltageFromTorque(
              5.0, DriveTransmission(properties).CalculateTorque(5.0, 12.0)),
      12.0, 1e-3);
}

TEST(DrivetrainModel, Kinematics) {
  DrivetrainModel::Properties properties;
  properties.wheelbase_radius = 0.5;
  properties.wheel_radius = 3 * 0.0254;
  DrivetrainModel model(properties, DriveTransmission({}),
                        DriveTransmission({}));
  EXPECT_NEAR((model.ForwardKinematics(Eigen::Vector2d(1.0, 1.0)) -
               Eigen::Vector2d(1.0, 0.0))
                  .lpNorm<Eigen::Infinity>(),
              0, 1e-3);
  EXPECT_NEAR((model.ForwardKinematics(Eigen::Vector2d(0.0, 1.0)) -
               Eigen::Vector2d(0.5, 1.0))
                  .lpNorm<Eigen::Infinity>(),
              0, 1e-3);
  EXPECT_NEAR((model.ForwardKinematics(Eigen::Vector2d(-1.0, 1.0)) -
               Eigen::Vector2d(0.0, 2.0))
                  .lpNorm<Eigen::Infinity>(),
              0, 1e-3);

  EXPECT_NEAR((model.InverseKinematics(
                   model.ForwardKinematics(Eigen::Vector2d(1.0, 1.0))) -
               Eigen::Vector2d(1.0, 1.0))
                  .lpNorm<Eigen::Infinity>(),
              0, 1e-3);
  EXPECT_NEAR((model.InverseKinematics(
                   model.ForwardKinematics(Eigen::Vector2d(0.0, 1.0))) -
               Eigen::Vector2d(0.0, 1.0))
                  .lpNorm<Eigen::Infinity>(),
              0, 1e-3);
  EXPECT_NEAR((model.InverseKinematics(
                   model.ForwardKinematics(Eigen::Vector2d(-1.0, 1.0))) -
               Eigen::Vector2d(-1.0, 1.0))
                  .lpNorm<Eigen::Infinity>(),
              0, 1e-3);
}

TEST(DrivetrainModelDynamics, AccelFromStandstillNoStiction) {
  DrivetrainModel::Properties properties;
  properties.wheelbase_radius = 0.5;
  properties.angular_drag = 0.0;
  properties.mass = 50.0;
  properties.moment_inertia = 0.5 * properties.mass *
                              properties.wheelbase_radius *
                              properties.wheelbase_radius;
  properties.force_stiction = 0.0;
  properties.wheel_radius = 3 * 0.0254;

  DriveTransmission::Properties trans_properties;
  {
    const double i_stall = 134;
    const double t_stall = 2.34;
    const double i_free = 4.7;
    const double w_free = 5500 * (M_PI / 30.0);
    trans_properties.motor_kt = t_stall / i_stall;
    trans_properties.motor_resistance = 12.0 / i_stall;
    trans_properties.motor_kv =
        (12.0 - i_free * trans_properties.motor_resistance) / w_free;
    trans_properties.gear_ratio = 1 / 4.5;
    trans_properties.num_motors = 2;
  }

  DrivetrainModel model(properties, DriveTransmission(trans_properties),
                        DriveTransmission(trans_properties));

  Eigen::Vector2d initial_velocity(0.0, 0.0);

  EXPECT_NEAR(DriveTransmission(trans_properties).CalculateTorque(0.0, 12.0),
              2 * 2.34 * 4.5, 1e-3);

  Eigen::Vector2d acceleration_result = model.ForwardDynamics(
      initial_velocity, Eigen::Vector2d(12.0, 12.0), false);
  EXPECT_NEAR(acceleration_result(0),
              4 * 2.34 * 4.5 / properties.wheel_radius / properties.mass, 1e-3);
  EXPECT_NEAR(acceleration_result(1), 0, 1e-3);

  Eigen::Vector2d voltage_result =
      model.InverseDynamics(initial_velocity, acceleration_result, false);
  EXPECT_NEAR(voltage_result(0), 12.0, 1e-3);
  EXPECT_NEAR(voltage_result(1), 12.0, 1e-3);
}

TEST(DrivetrainModelDynamics, SteadyStateForwards) {
  DrivetrainModel::Properties properties;
  properties.wheelbase_radius = 0.42;
  properties.angular_drag = -35;
  properties.mass = 50.0;
  properties.moment_inertia = 0.5 * properties.mass *
                              properties.wheelbase_radius *
                              properties.wheelbase_radius;
  properties.force_stiction = 20.0;
  properties.wheel_radius = 3.25 / 2 * 0.0254;

  DriveTransmission::Properties trans_properties;
  const double i_stall = 131;
  const double t_stall = 2.41;
  const double i_free = 0;
  const double w_free = 5330 * (2 * M_PI / 60.0);
  {
    trans_properties.motor_kt = t_stall / i_stall;
    trans_properties.motor_resistance = 12.0 / i_stall;
    trans_properties.motor_kv =
        (12.0 - i_free * trans_properties.motor_resistance) / w_free;
    trans_properties.gear_ratio = 1 / 4.16;
    trans_properties.num_motors = 2;
  }

  DrivetrainModel model(properties, DriveTransmission(trans_properties),
                        DriveTransmission(trans_properties));

  Eigen::Vector2d velocity(0.0, 0.0);

  for (int i = 0; i < 400; i++) {
    velocity += 0.1 * model.ForwardDynamics(velocity,
                                            Eigen::Vector2d(12.0, 12.0), false);
  }

  EXPECT_NEAR(velocity(0),
              w_free * trans_properties.gear_ratio * properties.wheel_radius,
              1e-1);

  velocity = Eigen::Vector2d(0.0, 0.0);

  for (int i = 0; i < 400; i++) {
    velocity += 0.1 * model.ForwardDynamics(
                          velocity, Eigen::Vector2d(-12.0, 12.0), false);
  }

  std::cout << velocity(1) << std::endl;
}

TEST(DrivetrainModelDynamics, AccelerationBounds) {
  DrivetrainModel::Properties properties;
  properties.wheelbase_radius = 0.42;
  properties.angular_drag = -35;
  properties.mass = 50.0;
  properties.moment_inertia = 0.5 * properties.mass *
                              properties.wheelbase_radius *
                              properties.wheelbase_radius;
  properties.force_stiction = 20.0;
  properties.wheel_radius = 3.25 / 2 * 0.0254;

  DriveTransmission::Properties trans_properties;
  const double i_stall = 131;
  const double t_stall = 2.41;
  const double i_free = 0;
  const double w_free = 5330 * (2 * M_PI / 60.0);
  {
    trans_properties.motor_kt = t_stall / i_stall;
    trans_properties.motor_resistance = 12.0 / i_stall;
    trans_properties.motor_kv =
        (12.0 - i_free * trans_properties.motor_resistance) / w_free;
    trans_properties.gear_ratio = 1 / 4.16;
    trans_properties.num_motors = 2;
  }

  DrivetrainModel model(properties, DriveTransmission(trans_properties),
                        DriveTransmission(trans_properties));

  // Zero curvature
  Bounds bounds_forwards = model.CalculateMinMaxAcceleration(
      Eigen::Vector2d(0.0, 0.0), 0.0, 12.0, true);

  // Positive curvature
  Bounds bounds_turn_ccw = model.CalculateMinMaxAcceleration(
      Eigen::Vector2d(0.0, 0.0), 2.0, 12.0, true);

  // Negative curvature
  Bounds bounds_turn_cw = model.CalculateMinMaxAcceleration(
      Eigen::Vector2d(0.0, 0.0), -2.0, 12.0, true);

  Eigen::Vector2d voltage_forwards = model.InverseDynamics(
      Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(bounds_forwards.max, 0), true);

  std::cout << voltage_forwards << std::endl;

  std::cout << bounds_forwards.max << "\t" << bounds_turn_ccw.max << "\t"
            << bounds_turn_cw.max << std::endl;
}

}  // namespace control
}  // namespace muan
