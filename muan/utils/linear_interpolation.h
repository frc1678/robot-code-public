#ifndef MUAN_UTILS_LINEAR_INTERPOLATION_
#define MUAN_UTILS_LINEAR_INTERPOLATION_

#include <vector>
#include <stdexcept>

namespace muan {

template <typename T>
class LinearInterpolation {
 public:
  LinearInterpolation(std::vector<double> x_set, std::vector<T> y_set) {
    if(x_set.size() != y_set.size()) {
      throw std::invalid_argument("There must be the same number of x-values and y-values");
    }
    if(x_set.size() < 2) {
      throw std::invalid_argument("Interpolate requires 2 or more control points");
    }
    x_set_ = x_set;
    y_set_ = y_set;
  }
  T operator()(double x) {
    int lower_bounds_index = -1;
    int upper_bounds_index = -1;
    for(int i = 0; i < x_set_.size(); i++) {
      if(x_set_[i] <= x && (lower_bounds_index == -1 || x_set_[lower_bounds_index] < x_set_[i])) {
        lower_bounds_index = i;
      }
      if(x_set_[i] >= x && (upper_bounds_index == -1 || x_set_[upper_bounds_index] > x_set_[i])) {
        upper_bounds_index = i;
      }
    }
    if(lower_bounds_index == -1 || upper_bounds_index == -1) {
      throw std::domain_error("An interpolation is only defined between the lowest and highest x-values");
    }
    double x0 = x_set_[lower_bounds_index];
    double x1 = x_set_[upper_bounds_index];
    T y0 = y_set_[lower_bounds_index];
    T y1 = y_set_[upper_bounds_index];

    // Obtained as the solution to (y-y0)/(x-x0) = (y1-y0)/(x1-x0)
    T y;
    if(x0 == x1) {
      y = y0;
    } else {
      y = y0 + (x - x0) * (y1 - y0) / (x1 - x0);
    }
    return y;
  }
 protected:
  std::vector<double> x_set_;
  std::vector<T> y_set_;
};

} // muan

#endif // MUAN_UTILS_LINEAR_INTERPOLATION_
