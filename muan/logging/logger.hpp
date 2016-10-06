#ifndef MUAN_LOGGING_LOGGER_HPP_
#define MUAN_LOGGING_LOGGER_HPP_

#include "logger.h"

namespace muan {
namespace logging {

Logger::Logger() : muan::Updateable(200 * muan::units::hz) {
  writer_ = std::make_shared<FileWriter>();
}

Logger::Logger(std::shared_ptr<FileWriter> writer) : muan::Updateable(200 * muan::units::hz) {
  writer_ = writer;
}

template <class T>
void Logger::AddQueue(std::string name, T& queue_reader) { //TODO(Wesley) Queues with same name
  QueueLog queue_log = {std::make_unique<Reader<T>>(queue_reader), name, name + ".csv"};
  queue_logs_.push_back(std::make_unique<QueueLog>(std::move(queue_log)));
}

void Logger::Update(muan::units::Time dt) {
  for (auto const& log : queue_logs_) {
    std::experimental::optional<std::string> message;
    while (message = log->reader->GetMessageAsCSV()) {
      writer_->WriteLine(log->filename, message.value());
    }
  }
  for (auto const& log : text_logs_) {
    std::experimental::optional<std::string> message;
    while (message = log.queue->ReadMessage()) {
      writer_-> WriteLine(log.filename, message.value());
    }
  }
}

TextLogger Logger::GetTextLogger(std::string name) { //TODO(Wesley) logs with same name?
  auto queue_ptr = std::make_shared<TextLogger::TextQueue>();
  auto queue_reader = std::make_shared<TextLogger::TextQueue::QueueReader>(queue_ptr->MakeReader());
  TextLog log_obj = {queue_reader, name, name + ".log"};
  text_logs_.push_back(log_obj);
  TextLogger logger(queue_ptr);
  return logger;
}

template <class T>
std::experimental::optional<std::string> Logger::Reader<T>::GetMessageAsCSV() {
  auto message = reader_.ReadMessage();
  if (message) {
    return muan::util::ProtoToCSV(*message);
  } else {
    return std::experimental::nullopt;
  }
}

template <class T>
Logger::Reader<T>::Reader(T& reader) : reader_{reader} {}

}  // namespace logging
}  // namespace muan

#endif /* MUAN_LOGGING_LOGGER_HPP_ */
