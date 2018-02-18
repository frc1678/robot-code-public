#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest_prod.h"
#include "muan/logging/filewriter.h"
#include "muan/logging/logger.h"
#include "muan/logging/textlogger.h"
#include "muan/queues/message_queue.h"
#include "muan/units/units.h"
#include "muan/utils/proto_utils.h"
#include "muan/utils/threading_utils.h"
#include "third_party/aos/common/time.h"
#include "third_party/aos/common/util/phased_loop.h"
#include "third_party/aos/linux_code/init.h"
#include "third_party/optional/optional.hpp"

namespace muan {
namespace logging {

Logger::Logger()
    : writer_{std::make_unique<FileWriter>()},
      textlog_reader_{text_logger.MakeReader()} {}
Logger::Logger(std::unique_ptr<FileWriter>&& writer)
    : writer_{std::move(writer)}, textlog_reader_{text_logger.MakeReader()} {}

void Logger::operator()() {
  aos::time::PhasedLoop phased_loop(std::chrono::milliseconds(20));

  running_ = true;

  while (running_) {
    Update();

    phased_loop.SleepUntilNext();
  }
}

void Logger::Update() {
  for (const auto& log : queue_logs_) {
    std::experimental::optional<std::string> message;
    while ((message = log->reader->GetMessageAsCSV(log->write_header))) {
      if (log) {
        if (log->write_header) {
          log->write_header = false;
        }
        writer_->WriteLine(log->filename, message.value());
      }
    }
  }
  std::experimental::optional<TextLogger::LogCall> message;
  while ((message = textlog_reader_.ReadMessage())) {
    message->message(writer_->GetTextFile(*message->thread_name + ".log"));
  }
  writer_->FlushAllFiles();
}

void Logger::Start() { running_ = true; }

void Logger::Stop() { running_ = false; }

TextLogger Logger::text_logger;

}  // namespace logging
}  // namespace muan
