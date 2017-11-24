#include <cstdio>
#include <utility>

namespace muan {
namespace logging {

template<typename... Ts>
void TextLogger::LogStream(const char* filename, int line, Ts... args) {
  // Get time and thread when function is called.
  // Lambda is called later in a different thread.
  uint64_t time = muan::utils::Timestamp();
  LogCall call;
  call.thread_name = &muan::utils::GetCurrentThreadName();
  call.message = [=](std::ostream& out){
    Stamp(out, time, filename, line);
    VariadicStreamWrite(out, args..., "\n");
  };
  log_calls_.WriteMessage(call);
}

template<typename... Ts>
void TextLogger::LogPrintf(const char* filename, int line, Ts... args) {
  // Get time and thread when function is called.
  // Lambda is called later in a different thread.
  uint64_t time = muan::utils::Timestamp();
  LogCall call;
  call.thread_name = &muan::utils::GetCurrentThreadName();
  call.message = [=](std::ostream& out){
    Stamp(out, time, filename, line);
    char buf[1024];  // More than enough for any reasonable message
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wformat-nonliteral"
#pragma clang diagnostic ignored "-Wformat-security"
#endif
    std::snprintf(buf, sizeof(buf), args...);
#ifdef __clang__
#pragma clang diagnostic pop
#endif
    out << buf << "\n";
  };
  log_calls_.WriteMessage(call);
}

template<typename T, typename... Ts>
void VariadicStreamWrite(std::ostream& out, T&& head, Ts&&... tail) {
  out << std::forward<T>(head);
  VariadicStreamWrite(out, std::forward<Ts>(tail)...);
}

template<typename T>
void VariadicStreamWrite(std::ostream& out, T&& head) {
  out << std::forward<T>(head);
}

}  // namespace logging
}  // namespace muan
