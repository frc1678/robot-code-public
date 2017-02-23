#include "c2017/webdash/server.h"

namespace c2017 {
namespace webdash {

// Dumb hack that will start to get really unwieldy as soon as we try to do
// anything useful.
// TODO(Wesley) Switch to json or ProtoBuf.js
void WebDashController::save(Mongoose::Request &request, StreamResponse &response) {
  if (request.get("auto") != "") {
    WebDashProto output_proto;
    int auto_mode = std::stoi(request.get("auto"));
    if (WebDash_Auto_IsValid(auto_mode)) {
      output_proto->set_auto_mode(WebDash_Auto(auto_mode));
      QueueManager::GetInstance().webdash_queue().WriteMessage(output_proto);
      response << "OK" << std::endl;
    } else {
      response << "ERR" << std::endl
               << "Auto mode value is out of bounds" << std::endl;
    }
  } else {
    response << "ERR" << std::endl
             << "No auto mode passed in" << std::endl;
  }
}

void WebDashController::setup() { addRoute("GET", "/save", WebDashController, save); }

void WebDashRunner::operator()() {
  server_.registerController(&controller_);
  server_.setOption("document_root", "c2017/webdash/www/");
  server_.start();

  while (true) {
    c2017::webdash::WebDashController::sleep(10);
  }
}

}  // namespace webdash
}  // namespace c2017
