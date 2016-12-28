#include "textlogger.h"

namespace muan {
namespace logging {

void TextLogger::operator () (const char *text) {
  auto time = aos::time::Time::Now().ToMSec();
  snprintf(&buffer_[0], 1024, "%ld,%s", time, text);
  log_queue_->WriteMessage(buffer_);
}

}
}
