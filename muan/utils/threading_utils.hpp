#include <algorithm>
#include <cstring>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

namespace muan {
namespace utils {

template<typename T, size_t N, typename... Ts1> template<typename F, typename... Ts0>
DeferCall<T, N, Ts1...>::DeferCall(F f, Ts0... args0) {
  // Capture args to curry function. The type depends on the values captured.
  auto call = [=](Ts1... args1) -> T { return f(args0..., args1...); };
  // Fail if the call can't be stored, otherwise copy to buf
  static_assert(sizeof(call) <= N, "Not enough space to store call");
  std::memcpy(buf, &call, sizeof(call));
  // Can't capture anything since it must be constexpr. Decltype doesn't
  // count as capturing. Remembers the type of call so it can be
  // reconstructed and called from buf.
  func = [](const DeferCall<T, N, Ts1...>* defer_call, Ts1... args1) -> T {
    return (*reinterpret_cast<const decltype(call)*>(defer_call->buf))(args1...);
  };
}

}  // namespace utils
}  // namespace muan
