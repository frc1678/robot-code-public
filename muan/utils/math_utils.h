#ifndef MUAN_UTILS_MATH_UTILS_H_
#define MUAN_UTILS_MATH_UTILS_H_

#include "Eigen/Core"
#include <iostream>
#include <random>

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

// Keep one random number generator per thread, as the implementation isn't
// thread-safe. "extern" because it's defined in math_utils.cpp
extern thread_local std::mt19937_64 rng;  // NOLINT

// Generate a single scalar value of gaussian noise
double GaussianNoise(double std_dev = 1.0, double mean = 0.0);

// Generate a vector of gaussian noise with a given covariance matrix
template <uint32_t A>
Eigen::Matrix<double, A, 1> GaussianNoise(
    const Eigen::Matrix<double, A, A>& covariance,
    const Eigen::Matrix<double, A, 1> mean =
        Eigen::Matrix<double, A, 1>::Zero()) {
  Eigen::Matrix<double, A, 1> ret;

  for (uint32_t i = 0; i < A; i++) {
    ret[i] = GaussianNoise();
  }

  return covariance * ret + mean;
}

// Perform a modulo operation that is relative to negative infinity, not 0
uint32_t true_modulo(int a, int b);

}  // namespace muan

#endif /* MUAN_UTILS_MATH_UTILS_H_ */
