#include "c2017/webdash/server.h"

namespace c2017 {
namespace webdash {

WebDashQueueWrapper& WebDashQueueWrapper::GetInstance() {
  static WebDashQueueWrapper instance;
  return instance;
}

AutoSelectionQueue& WebDashQueueWrapper::auto_selection_queue() { return auto_selection_queue_; }

LogNameQueue& WebDashQueueWrapper::log_name_queue() { return log_name_queue_; }

// Dumb hack that will start to get really unwieldy as soon as we try to do
// anything useful.
// TODO(Wesley) Switch to json or ProtoBuf.js
struct AutoChangeHandler : seasocks::WebSocket::Handler {
  void onConnect(seasocks::WebSocket * /*socket*/) override {}

  void onData(seasocks::WebSocket * /*socket*/, const char *data) override {
    AutoSelectionProto output_proto;
    int auto_mode = std::atoi(data);
    if (AutoSelection_Auto_IsValid(auto_mode)) {
      output_proto->set_auto_mode(AutoSelection_Auto(auto_mode));
      WebDashQueueWrapper::GetInstance().auto_selection_queue().WriteMessage(output_proto);
    }
  }

  void onDisconnect(seasocks::WebSocket * /*socket*/) override {}
};

void WebDashRunner::DataRequestHandler::onConnect(seasocks::WebSocket *con) {
    cons_.insert(con);
}

void WebDashRunner::DataRequestHandler::onDisconnect(seasocks::WebSocket *con) {
    cons_.erase(con);
}

void WebDashRunner::DataRequestHandler::onData(seasocks::WebSocket *con, const char * /*data*/) {
  std::vector<std::string> queue_entry_list;
  for (auto &queue_log : *queue_logs_) {
    queue_entry_list.push_back(
      "\"" + queue_log->name + "\": " + queue_log->reader->GetMessageAsJSON().value_or("{}"));
  }

  std::stringstream output_json;
  output_json << "{\n";
  for (size_t i = 0; i < queue_entry_list.size(); ++i) {
    if (i != 0) {
      output_json << ",";
    }
    output_json << queue_entry_list[i];
  }
  output_json << "}";

  con->send(output_json.str().c_str());
}

void WebDashRunner::operator()() {
  auto logger = std::make_shared<seasocks::PrintfLogger>();
  seasocks::Server server{logger};
  server.addWebSocketHandler("/save", std::make_shared<AutoChangeHandler>());
  server.addWebSocketHandler("/data", std::make_shared<DataRequestHandler>(queue_logs_));
  server.serve("c2017/webdash/www/", 5801);
}

}  // namespace webdash
}  // namespace c2017
