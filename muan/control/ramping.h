#ifndef MUAN_CONTROL_RAMPING_H_
#define MUAN_CONTROL_RAMPING_H_

namespace muan {

namespace control {

class Ramping {
 public:
  explicit Ramping(double acceleration, double initial = 0.0, bool deccelerate = false);
  double Update(double goal);

 private:
  double acceleration_;
  double profiled_goal_, unprofiled_goal_;
  bool deccelerate_;
};

}  // namespace control

}  // namespace muan

#endif  // MUAN_CONTROL_RAMPING_H_
