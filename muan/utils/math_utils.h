#ifndef MUAN_UTILS_MATH_UTILS_H_
#define MUAN_UTILS_MATH_UTILS_H_

#include "Eigen/Core"
#include <iostream>

namespace muan {

inline double Cap(double val, double min, double max) {
  double ret = val;
  if (val < min) {
    ret = min;
  } else if (val > max) {
    ret = max;
  }
  return ret;
}

template <int A, int B>
inline Eigen::Matrix<double, A, B> CapMatrix(
    const Eigen::Matrix<double, A, B>& val,
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

}  // namespace muan

#endif /* MUAN_UTILS_MATH_UTILS_H_ */
