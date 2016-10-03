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
void Logger::AddQueue(std::string name, T& queue_reader) {
  QueueLog queue_log = {std::make_unique<Reader<T>>(queue_reader), name};
  all_logs_.push_back(std::make_unique<QueueLog>(std::move(queue_log)));
}

void Logger::Update(muan::units::Time dt) {
  for (auto const& log : all_logs_) {
    std::experimental::optional<std::string> message;
    while (message = log->reader->GetMessageAsCSV()) {
      writer_->WriteLine(log->name, message.value_or(""));
    }
  }
}

std::experimental::optional<std::string> Logger::GenericReader::GetMessageAsCSV() {
  return std::experimental::nullopt;  // This should *never* happen
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
