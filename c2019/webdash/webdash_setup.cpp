#include "c2019/webdash/webdash_setup.h"
#include <fstream>
#include <string>
#include <vector>

namespace c2019 {

using muan::queues::webdash;

void StartWebdash() {
  // Video Stream

  webdash->AddVideoStream("?action=stream");

  // Display Object
  std::ifstream display_stream("c2019/webdash/display_object.json");

  std::string display_object((std::istreambuf_iterator<char>(display_stream)),
                             std::istreambuf_iterator<char>());

  webdash->DisplayObjectMaker(display_object);

  display_stream.close();
}

}  // namespace c2019
