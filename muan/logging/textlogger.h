#ifndef MUAN_LOGGING_TEXTLOGGER_H_
#define MUAN_LOGGING_TEXTLOGGER_H_

#include <ostream>
#include <string>
#include "muan/queues/message_queue.h"
#include "muan/utils/threading_utils.h"

namespace muan {
namespace logging {

class TextLogger {
 public:
  // Log without format strings, such as
  // LogStream(__FILE__, __LINE__, "x=", x, " y=", y);
  template<typename... Ts>
  void LogStream(const char* filename, int line, Ts... args);

  // Log with format strings, such as
  // LogPrintf(__FILE__, __LINE__, "x=%d y=%f", x, y);
  template<typename... Ts>
  void LogPrintf(const char* filename, int line, Ts... args);

  struct LogCall {
    muan::utils::DeferCall<void, 1024, std::ostream&> message;
    const std::string* thread_name;
  };

  using LogQueue = muan::queues::MessageQueue<LogCall, 500>;

  LogQueue::QueueReader MakeReader();

 private:
  // Write a stamp that traces where and when the log is from
  void Stamp(std::ostream& out, uint64_t time, const char* filename, int line);
  LogQueue log_calls_;
};

// Helper function to write a parameter pack to an ostream
template<typename T, typename... Ts>
void VariadicStreamWrite(std::ostream& out, T&& head, Ts&&... tail);
template<typename T>
void VariadicStreamWrite(std::ostream& out, T&& head);

}  // namespace logging
}  // namespace muan

#include "textlogger.hpp"

#endif  // MUAN_LOGGING_TEXTLOGGER_H_
