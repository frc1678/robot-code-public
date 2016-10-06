#include "textlogger.h"

namespace muan {
namespace logging {

void TextLogger::operator () (std::string text) {
  log_queue_->WriteMessage(text);
}

}
}
