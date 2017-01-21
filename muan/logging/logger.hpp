#ifndef MUAN_LOGGING_LOGGER_HPP_
#define MUAN_LOGGING_LOGGER_HPP_

#include "muan/logging/logger.h"

namespace muan {
namespace logging {

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
Logger::Reader<T>::Reader(T* reader) : reader_{reader} {}

}  // namespace logging
}  // namespace muan

#endif /* MUAN_LOGGING_LOGGER_HPP_ */
