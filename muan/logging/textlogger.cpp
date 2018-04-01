#ifndef __STDC_FORMAT_MACROS
#define __STDC_FORMAT_MACROS
#endif

#include "muan/logging/textlogger.h"

#include <inttypes.h>
#include <iostream>
#include <iterator>
#include <ostream>
#include <string>
#include <vector>

namespace muan {
namespace logging {

TextLogger::LogQueue::QueueReader TextLogger::MakeReader() {
  return log_calls_.MakeReader();
}

void TextLogger::Stamp(std::ostream& out, uint64_t time, int level,
                       const char* filename, int line) {
  switch (level) {
    case DEBUG:
      out << "[DEBUG]";
      break;
    case ERROR:
      out << "[ERROR]";
      break;
    case WARNING:
      out << "[WARNING]";
      break;
    case FATAL:
      out << "[FATAL]";
      break;
    case INFO:
      out << "[INFO]";
      break;
  }
  out << time << ":" << filename << ":" << line << ": ";
}

}  // namespace logging
}  // namespace muan
