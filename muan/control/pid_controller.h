#ifndef MUAN_CONTROL_PID_CONTROLLER_H_
#define MUAN_CONTROL_PID_CONTROLLER_H_

#include "third_party/unitscpp/unitscpp.h"
#include <type_traits>

namespace muan {

template <typename InputType, typename OutputType>

class PidController {
 public:
  using ProportionalConstant =
      typename std::remove_cv<decltype(OutputType(0) / InputType(0))>::type;
  using IntegralConstant = typename std::remove_cv<decltype(
      ProportionalConstant(0) / Time(0))>::type;
  using DerivativeConstant = typename std::remove_cv<decltype(
      ProportionalConstant(0) * Time(0))>::type;

  struct PidGains {
    ProportionalConstant kP;
    IntegralConstant kI;
    DerivativeConstant kD;
  };

  PidController(ProportionalConstant kP, IntegralConstant kI,
                DerivativeConstant kD)
      : kP(kP), kI(kI), kD(kD), integral_(0), last_proportional_(0) {}
  explicit PidController(const PidGains& gains)
      : PidController(gains.kP, gains.kI, gains.kD) {}

  OutputType Calculate(Time dt, InputType error) {
    return error * kP + CalculateIntegral(dt, error) * kI +
           (last_derivative_ = CalculateDerivative(dt, error)) * kD;
  }

  void SetProportionalConstant(ProportionalConstant p) { kP = p; }

  void SetIntegralConstant(IntegralConstant i) { kI = i; }

  void SetDerivativeConstant(DerivativeConstant d) { kD = d; }

  void SetGains(const PidGains& gains) {
    kP = gains.kP;
    kI = gains.kI;
    kD = gains.kD;
  }

  void Reset() {
    last_proportional_ = 0;
    integral_ = 0;
  }

  decltype(InputType(0) / s) GetDerivative() { return last_derivative_; }

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
  typename std::remove_cv<decltype(InputType(0) * s)>::type integral_;
  InputType last_proportional_;
  typename std::remove_cv<decltype(InputType(0) / s)>::type last_derivative_;
};

}  // namespace muan

#endif
