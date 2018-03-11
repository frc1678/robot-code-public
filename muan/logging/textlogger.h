#ifndef MUAN_LOGGING_TEXTLOGGER_H_
#define MUAN_LOGGING_TEXTLOGGER_H_

#include <ostream>
#include <string>

#include "muan/queues/message_queue.h"
#include "muan/utils/threading_utils.h"

constexpr int DEBUG = -1;
constexpr int ERROR = -2;
constexpr int WARNING = -3;
constexpr int FATAL = -4;
constexpr int INFO = -5;

namespace muan {
namespace logging {

class TextLogger {
 public:
  // Log with format strings, such as
  // Log(level, __FILE__, __LINE__, "x=%d y=%f", x, y);
  int level;
  template <typename... Ts>
  void Log(int level, const char* filename, int line, Ts... args);

  constexpr static size_t kBufferSize =
      1024;  // More than enough for any reasonable message

  struct LogCall {
    muan::utils::DeferCall<void, kBufferSize + 120, std::ostream&> message;
    const std::string* thread_name;
  };

  using LogQueue = muan::queues::MessageQueue<LogCall>;

  LogQueue::QueueReader MakeReader();

 private:
  // Write a stamp that traces where and when the log is from
  void Stamp(std::ostream& out, uint64_t time, int level, const char* filename,
             int line);
  LogQueue log_calls_{500};
};

}  // namespace logging
}  // namespace muan

#include "textlogger.hpp"

#endif  // MUAN_LOGGING_TEXTLOGGER_H_
