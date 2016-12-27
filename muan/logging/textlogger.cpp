#include "textlogger.h"

namespace muan {
namespace logging {

void TextLogger::operator () (const std::string &text) {
  auto time = aos::time::Time::Now().ToMSec();
  log_queue_->WriteMessage(std::to_string(time) + "," + text);
}

}
}
