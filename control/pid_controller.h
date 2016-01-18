#ifndef MUAN_CONTROL_PID_CONTROLLER_H_
#define MUAN_CONTROL_PID_CONTROLLER_H_

#include "../utils/timer.h"
#include "../unitscpp/unitscpp.h"

namespace muan {

template <typename InputType, typename OutputType>

class PidController {
 public:
  using ProportionalConstant =
      Units<OutputType::u1 - InputType::u1, OutputType::u2 - InputType::u2,
            OutputType::u3 - InputType::u3, OutputType::u4 - InputType::u4>;
  using IntegralConstant =
      Units<OutputType::u1 - InputType::u1, OutputType::u2 - InputType::u2 - 1,
            OutputType::u3 - InputType::u3, OutputType::u4 - InputType::u4>;
  using DerivativeConstant =
      Units<OutputType::u1 - InputType::u1, OutputType::u2 - InputType::u2 + 1,
            OutputType::u3 - InputType::u3, OutputType::u4 - InputType::u4>;
  PidController(ProportionalConstant kP, IntegralConstant kI,
                DerivativeConstant kD)
      : kP(kP), kI(kI), kD(kD), integral_(0), last_proportional_(0) {}

  OutputType Calculate(Time dt, InputType error) {
    auto proportional = error;
    auto derivative = (proportional - last_proportional_) / dt;
    integral_ += dt * error;
    last_proportional_ = proportional;
    return proportional * kP + integral_ * kI + derivative * kD;
  }

 private:
  InputType last_proportional_;
  Units<InputType::u1, InputType::u2 + 1, InputType::u3, InputType::u4>
      integral_;
  Units<OutputType::u1 - InputType::u1, OutputType::u2 - InputType::u2,
        OutputType::u3 - InputType::u3, OutputType::u4 - InputType::u4> kP;
  Units<OutputType::u1 - InputType::u1, OutputType::u2 - InputType::u2 - 1,
        OutputType::u3 - InputType::u3, OutputType::u4 - InputType::u4> kI;
  Units<OutputType::u1 - InputType::u1, OutputType::u2 - InputType::u2 + 1,
        OutputType::u3 - InputType::u3, OutputType::u4 - InputType::u4> kD;
};
}

#endif
