#ifndef MUAN_UTILS_MATH_UTILS_H_
#define MUAN_UTILS_MATH_UTILS_H_

#include "Eigen/Core"
#include <iostream>

namespace muan {

template <class T>
T Cap(T val, T min, T max) {
  T ret = val;
  if (val < min) {
    ret = min;
  } else if (val > max) {
    ret = max;
  }
  return ret;
}

template <int A, int B>
Eigen::Matrix<double, A, B> CapMatrix(const Eigen::Matrix<double, A, B>& val,
                                      const Eigen::Matrix<double, A, B>& min,
                                      const Eigen::Matrix<double, A, B>& max) {
  Eigen::Matrix<double, A, B> ret;
  for (uint32_t i = 0; i < A; i++) {
    for (uint32_t j = 0; j < B; j++) {
      ret(i, j) = Cap(val(i, j), min(i, j), max(i, j));
    }
  }
  return ret;
}

template <class T>
T abs(T val) {
  if (val < 0) val = -val;
  return val;
}

template <typename T>
using TimeDerivative = std::remove_cv_t<decltype(T{0} / s)>;

template <typename T>
using TimeDerivative2 = std::remove_cv_t<decltype(T{0} / s / s)>;

template <typename T>
using TimeIntegral = std::remove_cv_t<decltype(T{0} * s)>;

template <class T>
class Differentiator {
  private:
    T prev_val_ = T();
  public:
    TimeDerivative<T> differentiate(T val, Time dt) {
      TimeDerivative<T> derivative = (val - prev_val_) / dt;
      prev_val_ = val;
      return derivative;
    }
};

template <class T>
class Integrator {
  private:
    TimeIntegral<T> integral = TimeIntegral<T>();
  public:
    TimeIntegral<T> integrate(T val, Time dt) {
      integral += val * dt;
      return integral;
    }
    
    void reset() {
      integral = TimeIntegral<T>();
    }
};

} /* muan */

#endif
