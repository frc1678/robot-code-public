/*
 * TrapezoidalMotionProfile.h
 *
 *  Created on: Sep 25, 2015
 *      Author: Kyle
 */

#ifndef SRC_ROBOTCODE_TRAPEZOIDALMOTIONPROFILE_H_
#define SRC_ROBOTCODE_TRAPEZOIDALMOTIONPROFILE_H_

#include "motion_profile.h"
#include <cmath>

template<typename DistanceU>
class TrapezoidalMotionProfile : public MotionProfile<DistanceU>
{
  typedef Units<DistanceU::u1, DistanceU::u2-1, DistanceU::u3, DistanceU::u4> VelocityU;
  typedef Units<DistanceU::u1, DistanceU::u2-2, DistanceU::u3, DistanceU::u4> AccelerationU;
	Time _accel_time;
	Time _total_time;
	Time _deccel_time;
	VelocityU _max_speed;
	AccelerationU _max_acceleration;

public:
	TrapezoidalMotionProfile<DistanceU>(DistanceU distance, VelocityU max_speed, AccelerationU max_acceleration) {
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

			_accel_time = std::sqrt((2*accel_dist/max_acceleration)())*s;
			_deccel_time = _accel_time;
			_max_speed = _accel_time * _max_acceleration;
			_total_time = _accel_time + _deccel_time;
		}
	}
	virtual ~TrapezoidalMotionProfile() {}
	virtual VelocityU calculate_speed(Time time) override {
		VelocityU speed = 0;

		if (time <= _accel_time) {
			speed = _max_speed * (time / _accel_time);
		} else if (time <= _total_time - _deccel_time) {
			speed = _max_speed;
		} else if (time <= _total_time) {
			speed = _max_speed * ((_total_time - time) / _deccel_time);
		}

		return speed;
	}
	virtual DistanceU calculate_distance(Time time) override {
		DistanceU accel_dist = _max_speed * _accel_time / 2;
		DistanceU full_dist = _max_speed * (_total_time - _deccel_time - _accel_time);

		DistanceU distance = accel_dist + full_dist + _deccel_time * _max_speed / 2;

		if (time <= _accel_time) {
			distance = time * time * _max_speed / _accel_time / 2;
		} else if (time <= _total_time - _deccel_time) {
			distance = (time - _accel_time / 2) * _max_speed;
		} else if (time <= _total_time) {
			distance = accel_dist + full_dist + (_deccel_time + time - _total_time) * _max_speed / 2;
		}

		return distance;
	}
	bool finished(Time time) override
	{
		return time > _total_time;
	}
};

#endif /* SRC_ROBOTCODE_TRAPEZOIDALMOTIONPROFILE_H_ */
