#include "muan/utils/threading_utils.h"
#include <string>
#include "third_party/aos/linux_code/init.h"

namespace muan {
namespace utils {

int64_t Timestamp() {
  static int64_t start_time{ std::chrono::duration_cast<std::chrono::milliseconds>(
    aos::monotonic_clock::now() - aos::monotonic_clock::epoch()).count()};
  int64_t now{ std::chrono::duration_cast<std::chrono::milliseconds>(
    aos::monotonic_clock::now() - aos::monotonic_clock::epoch()).count()};
  // In some tests, mocktime is used and timestamp needs to be fixed
  if (now == 0) {
    start_time = 0;
  }
  return now - start_time;
}

void SetMocktimeEpoch() {
  aos::time::EnableMockTime(aos::monotonic_clock::epoch());
  // To ensure timestamp is set to 0 so tests get repeatable results,
  // call the function to update the static value
  Timestamp();
}

// Store the thread name instead of using prctl(PR_GET_NAME) since
// syscalls aren't realtime. Assume all naming of threads is done
// through muan::utils::SetCurrentThreadName.
thread_local std::string thread_name = "unnamed_thread";

void SetCurrentThreadName(const std::string& name) {
  aos::SetCurrentThreadName(name);
  thread_name = name;
}

const std::string& GetCurrentThreadName() {
  return thread_name;
}

}  // namespace utils
}  // namespace muan
