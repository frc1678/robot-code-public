#ifndef MUAN_UTILS_LINEAR_INTERPOLATION_
#define MUAN_UTILS_LINEAR_INTERPOLATION_

#include <vector>
#include <utility>
#include <functional>

namespace muan {
namespace utils {
  
template <typename T>
class LinearInterpolation {
 public:
  LinearInterpolation(std::vector<std::pair<double, T>> data);
  T operator()(double x) const;
  void AddControlPoint(std::pair<double, T> point);
  double lower_boundary() const;
  double upper_boundary() const;
 protected:
  std::vector<std::pair<double, T>> data_;
  // necessary for sorting
  std::function<bool(const std::pair<double, T>&, const std::pair<double, T>&)> compare_points_;
};

} // utils
} // muan


#include "linear_interpolation.hpp"

#endif // MUAN_UTILS_LINEAR_INTERPOLATION_
