#ifndef MUAN_CONTROL_PID_CONTROLLER_H_
#define MUAN_CONTROL_PID_CONTROLLER_H_

#include "../utils/timer.h"
#include "../unitscpp/unitscpp.h"

namespace muan {

template <typename InputType, typename OutputType>

class PidController {
 public:
  using ProportionalConstant = decltype(OutputType(0) / InputType(0));
  using IntegralConstant = decltype(ProportionalConstant(0) / s);
  using DerivativeConstant = decltype(ProportionalConstant(0) * s);
  PidController(ProportionalConstant kP, IntegralConstant kI,
                DerivativeConstant kD)
      : kP(kP), kI(kI), kD(kD), integral_(0), last_proportional_(0) {}

  OutputType Calculate(Time dt, InputType error) {
    return error * kP + CalculateIntegral(dt, error) * kI + CalculateDerivative(dt, error) * kD;
  }

  void SetProportionalConstants(ProportionalConstant p) { kP = p; }

  void SetIntegralConstant(IntegralConstant i) { kI = i; }

  void SetDerivativeConstant(DerivativeConstant d) { kD = d; }

  void Reset() {
    last_proportional_ = 0;
    integral_ = 0;
  }

 protected:
  decltype(InputType(0) / s) CalculateDerivative(Time dt, InputType error) {
    auto ret = (error - last_proportional_) / dt;
    last_proportional_ = error;
    return ret;
  }

  decltype(InputType(0) * s) CalculateIntegral(Time dt, InputType error) {
    integral_ += error * dt;
    return integral_;
  }

  ProportionalConstant kP;
  IntegralConstant kI;
  DerivativeConstant kD;
  Units<InputType::u1, InputType::u2 + 1, InputType::u3, InputType::u4>
      integral_;
  InputType last_proportional_;
};
}

#endif
