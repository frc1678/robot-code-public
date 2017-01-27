#ifndef MUAN_UTILS_HISTORY_H_
#define MUAN_UTILS_HISTORY_H_

#include <algorithm>
#include <vector>
#include <cmath>
#include "muan/units/units.h"
#include "third_party/aos/common/die.h"

namespace muan {
namespace utils {

template <class T>
class History {
 public:
  class Iterator {
   public:
    Iterator(const Iterator&) = default;
    Iterator& operator=(const Iterator&) = default;

    bool operator!=(const Iterator& other) { return position_ != other.position_; }

    Iterator operator++() {
      Iterator prev = *this;
      position_++;
      return prev;
    }
    T& operator*() { return hist_->hist_arr_[position_ % hist_->hist_arr_.size()]; }

   private:
    Iterator(int position, History* hist) : position_{position}, hist_{hist} {}

    int position_;
    History* hist_;

    friend class History;
  };

  explicit History(int size) : current_pos_(0) { hist_arr_.resize(static_cast<int>(size)); }

  void Update(T val) {
    hist_arr_[current_pos_ % hist_arr_.size()] = val;
    current_pos_++;
  }

  // Get the value from n cycles ago
  const T& GoBack(int n) {
    int read_pos = current_pos_ - n - 1;
    if (read_pos < earliest()) {
      ::aos::Die("Cannot go back to unrecorded history!");
    }

    return hist_arr_[read_pos % hist_arr_.size()];
  }

  Iterator begin() { return Iterator(earliest(), this); }

  Iterator end() { return Iterator(current_pos_, this); }

  int num_samples() { return current_pos_ - earliest(); }

  bool is_full() { return num_samples() == static_cast<int>(hist_arr_.size()); }

 private:
  // The cursor position (next to be written to)
  int current_pos_;

  // The position of the earliest element kept in history
  int earliest() { return std::max(0, current_pos_ - static_cast<int>(hist_arr_.size())); }

  std::vector<T> hist_arr_;

  friend class Iterator;
};

}  // namespace utils
}  // namespace muan

#endif  // MUAN_UTILS_HISTORY_H_
