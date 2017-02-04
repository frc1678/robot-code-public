#ifndef MUAN_LOGGING_LOGGER_HPP_
#define MUAN_LOGGING_LOGGER_HPP_

#include <memory>
#include <utility>
#include <string>
#include "muan/logging/logger.h"

namespace muan {
namespace logging {

template <class T>
void Logger::AddQueue(const std::string& name, T* queue) {
  for (const auto& log : queue_logs_) {
    if (log->name == name) {
      aos::Die("Two queues with same name \"%s\"", name.c_str());
    }
  }
  QueueLog queue_log = {std::make_unique<Reader<typename T::QueueReader>>(queue->MakeReader()), name,
                        name + ".csv", true};

  queue_logs_.push_back(std::make_unique<QueueLog>(std::move(queue_log)));
}

template <class R>
std::experimental::optional<std::string> Logger::Reader<R>::GetMessageAsCSV(bool header) {
  auto message = reader_.ReadMessage();
  if (message) {
    std::string out = "";
    if (header) {
      out = out + muan::util::ProtoToCsvHeader(*message.value().get()) + "\n";
    }
    out = out + muan::util::ProtoToCsv(*message.value().get());
    return out;
  } else {
    return std::experimental::nullopt;
  }
}

template <class R>
Logger::Reader<R>::Reader(R reader)
    : reader_{reader} {}

}  // namespace logging
}  // namespace muan

#endif  // MUAN_LOGGING_LOGGER_HPP_
