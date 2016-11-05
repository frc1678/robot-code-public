#ifndef MUAN_UTILS_LINEAR_INTERPOLATION_
#define MUAN_UTILS_LINEAR_INTERPOLATION_

#include <vector>
#include <utility>

namespace muan {

template <typename T>
class LinearInterpolation {
 public:
  LinearInterpolation(std::vector<std::pair<double, T>> data);
  T operator()(double x);
 protected:
  std::vector<std::pair<double, T>> data_;
};

} // muan

#include "linear_interpolation.hpp"

#endif // MUAN_UTILS_LINEAR_INTERPOLATION_
