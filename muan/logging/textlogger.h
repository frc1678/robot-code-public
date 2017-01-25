#ifndef MUAN_LOGGING_TEXTLOGGER_H_
#define MUAN_LOGGING_TEXTLOGGER_H_

#include <memory>
#include <string>
#include "muan/queues/message_queue.h"
#include "third_party/aos/common/time.h"

namespace muan {
namespace logging {

// TODO(Wesley) Use c style strings

class TextLogger {
 public:
  typedef muan::queues::MessageQueue<std::array<char, 1024>, 500> TextQueue;
  typedef std::shared_ptr<TextQueue> TextQueuePtr;
  TextLogger(TextQueuePtr log_queue) : log_queue_(log_queue) {}
  void operator()(const char *text);

 private:
  TextQueuePtr log_queue_;
  std::array<char, 1024> buffer_;
};

}  // namespace logging
} // namespace muan

#endif  // MUAN_LOGGING_TEXTLOGGER_H_
