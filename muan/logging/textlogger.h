#ifndef MUAN_LOGGING_TEXTLOGGER_H_
#define MUAN_LOGGING_TEXTLOGGER_H_

#include "muan/queues/message_queue.h"
#include <memory>
#include <string>

namespace muan {
namespace logging {

//TODO(Wesley) Use c style strings 

class TextLogger {
  public:
    typedef muan::queues::MessageQueue<std::string, 500> TextQueue;
    typedef std::shared_ptr<TextQueue> TextQueuePtr;
    TextLogger(TextQueuePtr log_queue) :
      log_queue_(log_queue) {}
    void operator () (std::string text);
  private:
    TextQueuePtr log_queue_;
};

}
}

#endif
