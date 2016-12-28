#ifndef MUAN_UTILS_HISTORY_H_
#define MUAN_UTILS_HISTORY_H_

#include <array>
#include "muan/units/units.h"
#include "third_party/aos/common/die.h"

namespace muan {
namespace utils {

template <class T, int size>
class History {
 public:
  explicit History(muan::units::Time time_step_) : current_pos_(0), time_step_(time_step_) {}

  void Update(T val) {
    hist_arr_[current_pos_] = val;
    current_pos_ = (current_pos_ + 1) % size;
  }

  const T& GoBack(muan::units::Time t) {
    if (t > time_step_ * size) {
      ::aos::Die("Cannot go back to unrecorded history!");
    }
    unsigned int element_pos =
        (current_pos_ - static_cast<int>(muan::units::convert(t, time_step_)) + size) % size;
    return hist_arr_[element_pos];
  }

  auto begin() { return begin(hist_arr_); }
  auto end() { return end(hist_arr_); }

 private:
  int current_pos_;
  muan::units::Time time_step_;
  std::array<T, size> hist_arr_;
};

}  // utils
}  // muan

#endif /* MUAN_UTILS_HISTORY_H_ */
