#ifndef MUAN_UTILS_MATH_UTILS_H_
#define MUAN_UTILS_MATH_UTILS_H_

namespace muan {

template<class T>
T Cap(T val, T min, T max) {
  if (val < min) {
    val = min;
  }else if (val > max) {
    val = max;
  }
  return val;
}

template<class T>
T abs(T val) {
  if (val < 0) val = -val;
  return val;
}

}

#endif
