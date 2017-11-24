#ifndef __STDC_FORMAT_MACROS
#define __STDC_FORMAT_MACROS
#endif
#include "muan/logging/textlogger.h"
#include <inttypes.h>

namespace muan {
namespace logging {

TextLogger::LogQueue::QueueReader TextLogger::MakeReader() {
  return log_calls_.MakeReader();
}

void TextLogger::Stamp(std::ostream& out, uint64_t time, const char* filename, int line) {
  out << time << ":" << filename << ":" << line << ": ";
}

}  // namespace logging
}  // namespace muan
