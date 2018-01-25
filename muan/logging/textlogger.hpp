#include <cstdio>
#include <utility>

namespace muan {
namespace logging {

template<typename... Ts>
void TextLogger::Log(const char* filename, int line, Ts... args) {
  // Get time and thread when function is called.
  // Lambda is called later in a different thread.
  uint64_t time = muan::utils::Timestamp();
  char buf[kBufferSize];
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wformat-nonliteral"
#pragma clang diagnostic ignored "-Wformat-security"
#endif
  std::snprintf(buf, sizeof(buf), args...);
#ifdef __clang__
#pragma clang diagnostic pop
#endif
  LogCall call;
  call.thread_name = &muan::utils::GetCurrentThreadName();
  call.message = [buf, time, filename, line, this](std::ostream& out){
    Stamp(out, time, filename, line);
    out << buf << "\n";
  };
  log_calls_.WriteMessage(call);
}

}  // namespace logging
}  // namespace muan
