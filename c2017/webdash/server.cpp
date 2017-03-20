#include "c2017/webdash/server.h"

namespace c2017 {
namespace webdash {

// Dumb hack that will start to get really unwieldy as soon as we try to do
// anything useful.
// TODO(Wesley) Switch to json or ProtoBuf.js
struct AutoChangeHandler : seasocks::WebSocket::Handler {
  void onConnect(seasocks::WebSocket * /*socket*/) override {}

  void onData(seasocks::WebSocket * /*socket*/, const char *data) override {
    WebDashProto output_proto;
    int auto_mode = std::atoi(data);
    if (WebDash_Auto_IsValid(auto_mode)) {
      output_proto->set_auto_mode(WebDash_Auto(auto_mode));
      QueueManager::GetInstance().webdash_queue().WriteMessage(output_proto);
    }
  }

  void onDisconnect(seasocks::WebSocket * /*socket*/) override {}
};

void WebDashRunner::operator()() {
  auto logger = std::make_shared<seasocks::PrintfLogger>();
  seasocks::Server server{logger};
  server.addWebSocketHandler("/save", std::make_shared<AutoChangeHandler>());
  server.serve("c2017/webdash/www/", 5801);
}

}  // namespace webdash
}  // namespace c2017
