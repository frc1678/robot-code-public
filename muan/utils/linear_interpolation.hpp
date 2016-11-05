#include "third_party/aos/common/die.h"

namespace muan {

template <typename T>
LinearInterpolation<T>::LinearInterpolation(std::vector<std::pair<double, T>> data) {
  if(data.size() < 2) {
    ::aos::Die("Interpolate requires 2 or more control points");
  }
  data_ = data;
}

template<typename T>
T LinearInterpolation<T>::operator()(double x) {
  // index of greatest x-value less than x
  int lower_bounds_index = -1;
  // index of lowest x-value greater than x
  int upper_bounds_index = -1;
  for(int i = 0; i < data_.size(); i++) {
    // If it is less than x, and either more than the current lower boundary
    // or a lower boundary has not been set, set the lower boundary to this.
    if(data_[i].first <= x && (lower_bounds_index == -1 || data_[lower_bounds_index].first < data_[i].first)) {
      lower_bounds_index = i;
    }
    // If it is more than x, and either less than the current upper boundary
    // or an upper boundary has not been set, set the upper boundary to this.
    if(data_[i].first >= x && (upper_bounds_index == -1 || data_[upper_bounds_index].first > data_[i].first)) {
      upper_bounds_index = i;
    }
  }
  if(lower_bounds_index == -1 || upper_bounds_index == -1) {
    ::aos::Die("An interpolation is only defined between the lowest and highest x-values");
  }
  double x0 = data_[lower_bounds_index].first;
  double x1 = data_[upper_bounds_index].first;
  T y0 = data_[lower_bounds_index].second;
  T y1 = data_[upper_bounds_index].second;

  // Obtained as the solution to (y-y0)/(x-x0) = (y1-y0)/(x1-x0)
  T y;
  if(x0 == x1) {
    y = y0;
  } else {
    y = y0 + (x - x0) * (y1 - y0) / (x1 - x0);
  }
  return y;
}

} // muan
