#include "muan/control/drivetrain_model.h"
#include <algorithm>
#include "muan/utils/math_utils.h"

namespace muan {
namespace control {

DriveTransmission::DriveTransmission(Properties properties) {
  dynamics_a_ = -properties.num_motors * properties.motor_kt *
                properties.motor_kv /
                (properties.motor_resistance * properties.gear_ratio *
                 properties.gear_ratio);
  dynamics_b_ = properties.num_motors * properties.efficiency *
                properties.motor_kt /
                (properties.motor_resistance * properties.gear_ratio);
}

double DriveTransmission::CalculateTorque(double velocity,
                                          double voltage) const {
  return dynamics_a_ * velocity + dynamics_b_ * voltage;
}

double DriveTransmission::VoltageFromTorque(double velocity,
                                            double torque) const {
  return (torque - dynamics_a_ * velocity) / dynamics_b_;
}

double DriveTransmission::dynamics_a() const { return dynamics_a_; }
double DriveTransmission::dynamics_b() const { return dynamics_b_; }
double DriveTransmission::gear_inertia() const { return gear_inertia_; }

DrivetrainModel::DrivetrainModel(DrivetrainModel::Properties properties,
                                 DriveTransmission low, DriveTransmission high)
    : wheelbase_radius_(properties.wheelbase_radius),
      angular_drag_(properties.angular_drag),
      mass_(properties.mass),
      moment_inertia_(properties.moment_inertia),
      force_stiction_(properties.force_stiction),
      force_friction_(properties.force_friction),
      wheel_radius_(properties.wheel_radius),
      transmission_low_(low),
      transmission_high_(high) {}

Eigen::Vector2d DrivetrainModel::ForwardKinematics(
    Eigen::Vector2d left_right) const {
  Eigen::Vector2d linear_angular;
  linear_angular(0) = (left_right(0) + left_right(1)) / 2.0;
  linear_angular(1) = (left_right(1) - left_right(0)) / (2 * wheelbase_radius_);
  return linear_angular;
}

Eigen::Vector2d DrivetrainModel::InverseKinematics(
    Eigen::Vector2d linear_angular) const {
  Eigen::Vector2d left_right;
  left_right(0) = linear_angular(0) - linear_angular(1) * wheelbase_radius_;
  left_right(1) = linear_angular(0) + linear_angular(1) * wheelbase_radius_;
  return left_right;
}

Eigen::Vector2d DrivetrainModel::ForwardDynamics(Eigen::Vector2d velocity,
                                                 Eigen::Vector2d voltage,
                                                 bool high_gear) const {
  const Eigen::Vector2d left_right_velocity = InverseKinematics(velocity);
  const DriveTransmission transmission =
      high_gear ? transmission_high_ : transmission_low_;

  Eigen::Vector2d robot_force_applied =
      Eigen::Vector2d(transmission.CalculateTorque(
                          left_right_velocity(0) / wheel_radius_, voltage(0)) /
                          wheel_radius_,
                      transmission.CalculateTorque(
                          left_right_velocity(1) / wheel_radius_, voltage(1)) /
                          wheel_radius_);

  // Angular drag: a fudge factor made up to (empirically) compensate for
  // skid.
  double robot_torque_angular_drag = angular_drag_ * velocity(1);
  Eigen::Vector2d robot_force_angular_drag =
      0.5 * Eigen::Vector2d(-robot_torque_angular_drag / wheelbase_radius_,
                            robot_torque_angular_drag / wheelbase_radius_);

  // Calculate static friction: if either side of the robot's drivetrain is
  // stationary, static friction is the opposite of the net force on that side
  // of the drivetrain, clipped to within an interval given by the maximum
  // static_friction.
  Eigen::Vector2d robot_force_stiction = Eigen::Vector2d::Zero();
  Eigen::Vector2d robot_force_friction = Eigen::Vector2d::Zero();
  if (std::abs(left_right_velocity(0)) < 1e-3) {
    robot_force_stiction(0) =
        muan::utils::Cap(-robot_force_applied(0) - robot_force_angular_drag(0),
                         -force_stiction_, force_stiction_);
  } else {
    robot_force_friction(0) =
        std::copysign(force_friction_, -left_right_velocity(0));
  }
  if (std::abs(left_right_velocity(1)) < 1e-3) {
    robot_force_stiction(1) =
        muan::utils::Cap(-robot_force_applied(1) - robot_force_angular_drag(1),
                         -force_stiction_, force_stiction_);
  } else {
    robot_force_friction(1) =
        std::copysign(force_friction_, -left_right_velocity(1));
  }

  // Net force on each side of the drivetrain
  const Eigen::Vector2d robot_force_net =
      robot_force_applied + robot_force_angular_drag + robot_force_stiction +
      robot_force_friction;

  // The gearing is down a ratio, so its mass has a disproportionate impact on
  // acceleration.
  Eigen::Vector2d acceleration_net;
  acceleration_net(0) = (robot_force_net(0) + robot_force_net(1)) / mass_;
  acceleration_net(1) = wheelbase_radius_ *
                        (robot_force_net(1) - robot_force_net(0)) /
                        moment_inertia_;
  return acceleration_net;
}

Eigen::Vector2d DrivetrainModel::InverseDynamics(Eigen::Vector2d velocity,
                                                 Eigen::Vector2d acceleration,
                                                 bool high_gear) const {
  const Eigen::Vector2d left_right_velocity = InverseKinematics(velocity);
  const Eigen::Vector2d left_right_acceleration =
      InverseKinematics(acceleration);

  const DriveTransmission transmission =
      high_gear ? transmission_high_ : transmission_low_;

  const double robot_force_forwards = acceleration(0) * mass_;
  const double robot_torque = acceleration(1) * moment_inertia_;

  // Attribute half of the torque and half of the force to each side.
  Eigen::Vector2d robot_force_net =
      0.5 *
      Eigen::Vector2d(robot_force_forwards - robot_torque / wheelbase_radius_,
                      robot_force_forwards + robot_torque / wheelbase_radius_);

  // Calculate the net torque on the robot from angular drag, and attribute half
  // of this to force on each side of the drivetrain.
  double angular_drag_torque = angular_drag_ * velocity(1);
  Eigen::Vector2d robot_force_angular_drag =
      0.5 * Eigen::Vector2d(-angular_drag_torque / wheelbase_radius_,
                            angular_drag_torque / wheelbase_radius_);

  // When either side of the robot isn't moving, apply the maximum stiction
  // possible in the opposite direction to the way we want to be accelerating.
  // This is instantaneously suboptimal - it yields more voltage than is
  // necessary at the very moment of initial acceleration, but is still correct.
  Eigen::Vector2d robot_force_stiction = Eigen::Vector2d::Zero();
  Eigen::Vector2d robot_force_friction = Eigen::Vector2d::Zero();
  if (std::abs(left_right_velocity(0)) < 1e-3) {
    robot_force_stiction(0) =
        -std::copysign(force_stiction_, left_right_acceleration(0));
  } else {
    robot_force_friction(0) =
        -std::copysign(force_friction_, left_right_acceleration(0));
  }
  if (std::abs(left_right_velocity(1)) < 1e-3) {
    robot_force_stiction(1) =
        -std::copysign(force_stiction_, left_right_acceleration(1));
  } else {
    robot_force_friction(1) =
        -std::copysign(force_friction_, left_right_acceleration(1));
  }

  const Eigen::Vector2d robot_force_applied =
      robot_force_net - robot_force_stiction - robot_force_friction -
      robot_force_angular_drag;

  const Eigen::Vector2d wheel_torque_applied =
      robot_force_applied * wheel_radius_;

  Eigen::Vector2d output;
  output(0) = transmission.VoltageFromTorque(
      left_right_velocity(0) / wheel_radius_, wheel_torque_applied(0));
  output(1) = transmission.VoltageFromTorque(
      left_right_velocity(1) / wheel_radius_, wheel_torque_applied(1));
  return output;
}

Bounds DrivetrainModel::CalculateMinMaxAcceleration(Eigen::Vector2d velocity,
                                                    double curvature,
                                                    double max_voltage,
                                                    bool high_gear) const {
  Bounds bounds;

  DriveTransmission transmission =
      high_gear ? transmission_high_ : transmission_low_;

  Eigen::Vector2d left_right_velocity = InverseKinematics(velocity);

  // The relationship between directionality on the left and right sides of the
  // drivetrain. 1 means that they are going in the same direction, -1 means
  // they move in opposite directions.
  int relationship_lr = (std::abs(curvature) < 1 / wheelbase_radius_) ? 1 : -1;

  // There are four possible cases that we want to encapsulate:
  //  - Left voltage is at full negative, |v_right| < |v_left|
  //  - Left voltage is at full positive, |v_right| < |v_left|
  //  - Right voltage is at full negative, |v_left| < |v_right|
  //  - Right voltage is at full positive, |v_left| < |v_right|
  //  Loop through all of the cases.
  for (int i = 0; i < 4; i++) {
    bool master_voltage_negative = i & 1;
    int master_index = (i >> 1) & 1;

    // Call one side of the drivetrain the "master" and the other the
    // "follower". The master will get full voltage (positive or negative), and
    // the follower will choose a voltage to match it based on curvature.

    int master_direction = 1;
    int follower_direction = relationship_lr * master_direction;

    Eigen::Vector2d robot_force_stiction = Eigen::Vector2d::Zero();
    Eigen::Vector2d robot_force_friction = Eigen::Vector2d::Zero();
    if (std::abs(left_right_velocity(master_index)) < 1e-3) {
      robot_force_stiction(master_index) = -master_direction * force_stiction_;
    } else {
      robot_force_friction(master_index) = -master_direction * force_friction_;
    }
    if (std::abs(left_right_velocity(!master_index)) < 1e-3) {
      robot_force_stiction(!master_index) =
          -follower_direction * force_stiction_;
    } else {
      robot_force_friction(!master_index) =
          -follower_direction * force_friction_;
    }

    double robot_torque_angular_drag = angular_drag_ * velocity(1);
    Eigen::Vector2d robot_force_angular_drag =
        0.5 * Eigen::Vector2d(-robot_torque_angular_drag / wheelbase_radius_,
                              robot_torque_angular_drag / wheelbase_radius_);

    Eigen::Vector2d robot_force_applied;
    robot_force_applied(master_index) =
        transmission.CalculateTorque(
            left_right_velocity(master_index),
            master_voltage_negative ? -max_voltage : max_voltage) /
        wheel_radius_;
    robot_force_applied(!master_index) =
        (wheelbase_radius_ * mass_ +
         (master_index ? 1 : -1) * curvature * moment_inertia_) /
            (wheelbase_radius_ * mass_ -
             (master_index ? 1 : -1) * curvature * moment_inertia_) *
            (robot_force_applied(master_index) +
             robot_force_angular_drag(master_index) +
             robot_force_stiction(master_index) +
             robot_force_friction(master_index)) -
        robot_force_angular_drag(!master_index) -
        robot_force_stiction(!master_index) -
        robot_force_friction(!master_index);

    Eigen::Vector2d robot_force_net =
        robot_force_applied + robot_force_angular_drag + robot_force_stiction +
        robot_force_friction;

    Eigen::Vector2d acceleration_net;
    acceleration_net(0) = (robot_force_net(0) + robot_force_net(1)) / mass_;
    acceleration_net(1) = wheelbase_radius_ *
                          (robot_force_net(1) - robot_force_net(0)) /
                          moment_inertia_;

    Eigen::Vector2d voltage_applied;
    voltage_applied(0) =
        transmission.VoltageFromTorque(left_right_velocity(0) / wheel_radius_,
                                       robot_force_net(0) * wheel_radius_);
    voltage_applied(1) =
        transmission.VoltageFromTorque(left_right_velocity(0) / wheel_radius_,
                                       robot_force_net(1) * wheel_radius_);

    // Otherwise this is an invalid config!
    if (std::abs(voltage_applied(!master_index)) <
        std::abs(voltage_applied(master_index)) + 1e-6) {
      bounds.max = std::max(bounds.max, acceleration_net(0));
      bounds.min = std::min(bounds.min, acceleration_net(0));
    }
  }

  return bounds;
}

}  // namespace control
}  // namespace muan
