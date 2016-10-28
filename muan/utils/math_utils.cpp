#include "math_utils.h"

namespace muan {

// Keep one random number generator per thread, as the implementation isn't
// thread-safe
thread_local std::mt19937_64 rng;  // NOLINT

double GaussianNoise(double std_dev, double mean) {
  std::normal_distribution<double> dist(mean, std_dev);
  return dist(rng);
}

}  // muan
