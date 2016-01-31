/*
 * TrapezoidalMotionProfile.h
 *
 *  Created on: Sep 25, 2015
 *      Author: Kyle
 */

#ifndef MUAN_CONTROL_TRAPEZOIDALMOTIONPROFILE_H_
#define MUAN_CONTROL_TRAPEZOIDALMOTIONPROFILE_H_

#include "motion_profile.h"
#include <cmath>

namespace muan {

template <typename DistanceU>
class TrapezoidalMotionProfile : public MotionProfile<DistanceU> {
  typedef Units<DistanceU::u1, DistanceU::u2 - 1, DistanceU::u3, DistanceU::u4>
      VelocityU;
  typedef Units<DistanceU::u1, DistanceU::u2 - 2, DistanceU::u3, DistanceU::u4>
      AccelerationU;
  Time _accel_time;
  Time _total_time;
  Time _deccel_time;
  VelocityU _max_speed;
  AccelerationU _max_acceleration;
  bool is_negative;
  DistanceU total_distance;

 public:
  TrapezoidalMotionProfile<DistanceU>(DistanceU distance, VelocityU max_speed,
                                      AccelerationU max_acceleration) {
    total_distance = distance;
    if (distance < DistanceU(0)) {
      distance = -distance;
      is_negative = true;
    } else
      is_negative = false;
    _accel_time = max_speed / max_acceleration;
    _deccel_time = max_speed / max_acceleration;
    _max_speed = max_speed;
    _max_acceleration = max_acceleration;

    if (distance > (_accel_time + _deccel_time) * _max_speed / 2) {
      _total_time = _accel_time + _deccel_time +
                    (distance - _max_speed * (_deccel_time + _accel_time) / 2) /
                        _max_speed;
    } else {
      DistanceU accel_dist = distance / 2;

      _accel_time = std::sqrt((2 * accel_dist / max_acceleration)()) * s;
      _deccel_time = _accel_time;
      _max_speed = _accel_time * _max_acceleration;
      _total_time = _accel_time + _deccel_time;
      std::cout << _total_time << ", " << _accel_time << ", " << _deccel_time
                << std::endl;
    }
  }
  virtual ~TrapezoidalMotionProfile() {}
  AccelerationU CalculateSecondDerivative(Time time) override {
    if (time < _accel_time) {
      return _max_acceleration;
    } else if (time < _total_time - _deccel_time) {
      return 0;
    } else if (time < _total_time) {
      return -_max_acceleration;
    } else {
      return 0;
    }
  }
  VelocityU CalculateDerivative(Time time) override {
    VelocityU speed = 0;

    if (time <= _accel_time) {
      speed = _max_speed * (time / _accel_time);
    } else if (time <= _total_time - _deccel_time) {
      speed = _max_speed;
    } else if (time <= _total_time) {
      speed = _max_speed * ((_total_time - time) / _deccel_time);
    }

    return is_negative ? -speed : speed;
  }
  DistanceU Calculate(Time time) override {
    DistanceU accel_dist = _max_speed * _accel_time / 2;
    DistanceU deccel_dist = _max_speed * _deccel_time / 2;
    DistanceU full_speed_dist =
        _max_speed * (_total_time - _deccel_time - _accel_time);

    DistanceU distance =
        accel_dist + full_speed_dist + _deccel_time * _max_speed / 2;

    if (time <= _accel_time) {
      distance = time * time * _max_speed / _accel_time / 2;
    } else if (time <= _total_time - _deccel_time) {
      distance = accel_dist + _max_speed * (time - _accel_time);
    } else if (time <= _total_time) {
      Time time_left = _total_time - time;
      DistanceU dist_left =
          time_left * time_left * _max_speed / _accel_time / 2;
      distance = accel_dist + full_speed_dist + (deccel_dist - dist_left);
    }

    return is_negative ? -distance : distance;
  }
  bool finished(Time time) override { return time > _total_time; }
  DistanceU GetTotalDistance() override { return total_distance; }
};
}

#endif /* SRC_ROBOTCODE_TRAPEZOIDALMOTIONPROFILE_H_ */
