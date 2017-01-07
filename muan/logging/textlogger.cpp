#ifndef __STDC_FORMAT_MACROS
#define __STDC_FORMAT_MACROS
#endif
#include "textlogger.h"
#include <inttypes.h>
namespace muan {
namespace logging {

void TextLogger::operator()(const char *text) {
  auto time = aos::time::Time::Now().ToMSec();
  snprintf(&buffer_[0], 1024, "%" PRId64 ",%s", time, text);
  log_queue_->WriteMessage(buffer_);
}
}
}
