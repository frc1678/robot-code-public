#ifndef MUAN_LOGGING_LOGGER_HPP_
#define MUAN_LOGGING_LOGGER_HPP_

#include "logger.h"

namespace muan {
namespace logging {

Logger::Logger() { writer_ = std::make_unique<FileWriter>(); }

Logger::Logger(std::unique_ptr<FileWriter>&& writer) : writer_(std::move(writer)) {}

template <class T>
void Logger::AddQueue(const std::string& name, T* queue_reader) {
  for (const auto& log : queue_logs_) {
    if (log->name == name) {
      aos::Die("Two queues with same name \"%s\"", name.c_str());
    }
  }
  QueueLog queue_log = {std::make_unique<Reader<T>>(queue_reader), name, name + ".csv"};

  queue_logs_.push_back(std::make_unique<QueueLog>(std::move(queue_log)));
}

void Logger::Run() {
  aos::time::PhasedLoop phased_loop(aos::time::Time::InMS(20));

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

template <class T>
std::experimental::optional<std::string> Logger::Reader<T>::GetMessageAsCSV() {
  auto message = reader_->ReadMessage();
  if (message) {
    return muan::util::ProtoToCSV(*message.value().get());
  } else {
    return std::experimental::nullopt;
  }
}

template <class T>
Logger::Reader<T>::Reader(T* reader)
    : reader_{reader} {}

}  // namespace logging
}  // namespace muan

#endif /* MUAN_LOGGING_LOGGER_HPP_ */
