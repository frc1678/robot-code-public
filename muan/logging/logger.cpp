#include "muan/logging/logger.h"

namespace muan {
namespace logging {

Logger::Logger() { writer_ = std::make_unique<FileWriter>(); }

Logger::Logger(std::unique_ptr<FileWriter>&& writer) : writer_(std::move(writer)) {}

void Logger::operator()() {
  aos::time::PhasedLoop phased_loop(std::chrono::milliseconds(20));

  aos::SetCurrentThreadRealtimePriority(20);
  aos::SetCurrentThreadName("Logger");

  running_ = true;

  while (running_) {
    Update();
    phased_loop.SleepUntilNext();
  }
}

void Logger::Update() {
  for (const auto& log : queue_logs_) {
    std::experimental::optional<std::string> message;
    while ((message = log->reader->GetMessageAsCSV())) {
      writer_->WriteLine(log->filename, message.value());
    }
  }
  for (const auto& log : text_logs_) {
    std::experimental::optional<std::array<char, 1024>> message;
    while ((message = log.queue->ReadMessage())) {
      writer_->WriteLine(log.filename, std::string(&message.value()[0]));
    }
  }
}

void Logger::Start() { running_ = true; }

void Logger::Stop() { running_ = false; }

TextLogger Logger::MakeTextLogger(const std::string& name) {  // TODO(Wesley) logs with same name?
  auto queue_ptr = std::make_shared<TextLogger::TextQueue>();
  auto queue_reader = std::make_shared<TextLogger::TextQueue::QueueReader>(queue_ptr->MakeReader());
  TextLog log_obj = {queue_reader, name, name + ".log"};
  text_logs_.push_back(log_obj);
  TextLogger logger(queue_ptr);
  return logger;
}

}  // namespace logging
}  // namespace muan
