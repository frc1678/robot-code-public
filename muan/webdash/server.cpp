#include "muan/webdash/server.h"
#include <string>
#include <vector>
#include "muan/logging/filewriter.h"

namespace muan {
namespace webdash {

void WebDashRunner::AddAutos(const std::vector<std::string>& more_autos) {
  auto_list_.insert(auto_list_.end(), more_autos.begin(), more_autos.end());
}

void WebDashRunner::AddVideoStream(std::string video_stream) {
  video_stream_list_.push_back(video_stream);
}

WebDashQueueWrapper& WebDashQueueWrapper::GetInstance() {
  static WebDashQueueWrapper instance;
  return instance;
}

AutoSelectionQueue& WebDashQueueWrapper::auto_selection_queue() {
  return auto_selection_queue_;
}

struct AutoChangeHandler : seasocks::WebSocket::Handler {
  void onConnect(seasocks::WebSocket* /*socket*/) override {}

  void onData(seasocks::WebSocket* /*socket*/, const char* data) override {
    AutoSelectionProto output_proto;

    output_proto->set_auto_mode(data);

    WebDashQueueWrapper::GetInstance().auto_selection_queue().WriteMessage(
        output_proto);
  }

  void onDisconnect(seasocks::WebSocket* /*socket*/) override {}
};

struct LogNameHandler : seasocks::WebSocket::Handler {
  void onConnect(seasocks::WebSocket* /* socket */) override {}
  void onDisconnect(seasocks::WebSocket* /* socket */) override {}

  void onData(seasocks::WebSocket* /*socket*/, const char* data) override {
    muan::logging::FileWriter::CreateReadableName(data);
  }
};

void WebDashRunner::DataRequestHandler::onData(seasocks::WebSocket* con,
                                               const char* /*data*/) {
  std::stringstream output_json;
  output_json << "{";
  for (size_t i = 0; i < queue_logs_->size(); ++i) {
    auto& queue_logs = *queue_logs_;
    if (i != 0) {
      output_json << ',';
    }
    output_json << "\"" << queue_logs[i]->name
                << "\":" +
                       queue_logs[i]->reader->GetMessageAsJSON().value_or("{}");
  }
  output_json << '}';
  con->send(output_json.str().c_str());
}

// Creates a vector in correct syntax from list of autos to be interpreted by
// json
void WebDashRunner::AutoListRequestHandler::onData(seasocks::WebSocket* con,
                                                   const char* /*data*/) {
  std::string json_auto_list = "[\"";
  bool first_auto = true;
  for (auto auto_name : auto_list_) {
    if (!first_auto) {
      json_auto_list += ", \"";
    } else {
      first_auto = false;
    }
    json_auto_list += auto_name + "\"";
  }
  json_auto_list += "]";
  con->send(json_auto_list.c_str());
}

void WebDashRunner::VideoListRequestHandler::onData(seasocks::WebSocket* con,
                                                    const char* /*data*/) {
  std::string json_video_list = "[\"";
  bool first_video = true;
  for (auto video_name : video_list_) {
    if (!first_video) {
      json_video_list += ", \"";
    } else {
      first_video = false;
    }
    json_video_list += video_name + "\"";
  }
  json_video_list += "]";
  con->send(json_video_list.c_str());
}

void WebDashRunner::WebDashModeRequestHandler::onData(seasocks::WebSocket* con,
                                                      const char* /*data*/) {
  if (mode_ == ROBORIO) {
    con->send("Roborio");
  } else if (mode_ == JETSON) {
    con->send("Jetson");
  } else {
    con->send("Undefined");
  }
}

void WebDashRunner::DisplayRequestHandler::onData(seasocks::WebSocket* con,
                                                  const char* /*data*/) {
  std::string json_display = "";
  json_display = *display_object_;
  con->send(json_display.c_str());
}

void WebDashRunner::DisplayObjectMaker(const std::string display_object) {
  display_object_ = display_object;
}

void WebDashRunner::operator()() {
  auto logger = std::make_shared<seasocks::PrintfLogger>();
  seasocks::Server server{logger};
  server.addWebSocketHandler("/auto", std::make_shared<AutoChangeHandler>());
  server.addWebSocketHandler(
      "/autolist", std::make_shared<AutoListRequestHandler>(auto_list_));
  server.addWebSocketHandler(
      "/videolist",
      std::make_shared<VideoListRequestHandler>(video_stream_list_));
  server.addWebSocketHandler("/logname", std::make_shared<LogNameHandler>());
  server.addWebSocketHandler("/data",
                             std::make_shared<DataRequestHandler>(queue_logs_));
  server.addWebSocketHandler(
      "/webdashmode", std::make_shared<WebDashModeRequestHandler>(kMode));
  server.addWebSocketHandler(
      "/display", std::make_shared<DisplayRequestHandler>(&display_object_));
  server.serve("muan/webdash/www/", 5801);
}

}  // namespace webdash
}  // namespace muan
