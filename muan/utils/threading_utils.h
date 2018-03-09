#ifndef MUAN_UTILS_THREADING_UTILS_H_
#define MUAN_UTILS_THREADING_UTILS_H_

#include <stdint.h>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include "third_party/aos/common/time.h"

namespace muan {
namespace utils {

// Class to store a function call to be evaluated later. For use
// when you can't use std::function because it isn't realtime.
// T - Return type
// N - Number of bytes to be used. Must be enough to store arguments.
// Ts - The arguments ommited from the constructor, to be called later
template <typename T, size_t N, typename... Ts1>
class DeferCall {
 public:
  // Defers the call f(args). If you call this with a pointer or
  // reference to something that will go out of scope, it will segfault
  // when evaluated, so don't do that.
  template <typename F, typename... Ts0>
  DeferCall(F f, Ts0... args0);
  DeferCall() { func = nullptr; }
  // Evaluate the call
  T operator()(Ts1... args1) const { return func(this, args1...); }

 protected:
  // The buffer where the function call is stored
  char buf[N];
  // How to interpret buf, needed since changing the type of args changes
  // the type stored in buf. Needs to be called with this since it must
  // be constexpr and therefore can't capture data.
  T (*func)(const DeferCall<T, N, Ts1...>*, Ts1...);
};

// Get timestamp
int64_t Timestamp();
// Wrapper around Aos function, sets timestamp to 0. Needed for tests that
// use mocktime and require a timestamp.
void SetMocktimeEpoch();
// Wrapper around Aos function, stores thread name so it can be read in realtime
void SetCurrentThreadName(const std::string& name);
const std::string& GetCurrentThreadName();

}  // namespace utils
}  // namespace muan

#include "muan/utils/threading_utils.hpp"

#endif  // MUAN_UTILS_THREADING_UTILS_H_
