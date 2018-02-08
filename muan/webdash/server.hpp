#ifndef MUAN_WEBDASH_SERVER_HPP_
#define MUAN_WEBDASH_SERVER_HPP_

#include "muan/webdash/server.h"

#include <memory>
#include <string>
#include <utility>
#include "muan/utils/proto_utils.h"

namespace muan {
namespace webdash {

template <class T>
void WebDashRunner::AddQueue(const std::string& name, T* queue) {
  for (const auto& log : queue_logs_) {
    if (log->name == name) {
      aos::Die("Two queues with same name \"%s\"", name.c_str());
    }
  }
  QueueLog queue_log = {
      std::make_unique<Reader<typename T::QueueReader>>(queue->MakeReader()),
      name};

  queue_logs_.push_back(std::make_unique<QueueLog>(std::move(queue_log)));
}

template <class R>
std::experimental::optional<std::string>
WebDashRunner::Reader<R>::GetMessageAsJSON() {
  auto message = reader_.ReadLastMessage();
  if (message) {
    std::stringstream output;
    muan::util::ProtoToJson(*message.value().get(), output);
    return output.str();
  } else {
    return std::experimental::nullopt;
  }
}

template <class R>
WebDashRunner::Reader<R>::Reader(R reader) : reader_{reader} {}

}  // namespace webdash
}  // namespace muan

#endif  // MUAN_WEBDASH_SERVER_HPP_
