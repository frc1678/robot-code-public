#include "o2018/webdash/webdash_setup.h"
#include <fstream>
#include <string>
#include <vector>

namespace o2018 {

using muan::queues::webdash;

void StartWebdash() {
  // Video Stream

  webdash->AddVideoStream("?action=stream");

  // Display Object
  std::ifstream display_stream("o2018/webdash/display_object.json");

  std::string display_object((std::istreambuf_iterator<char>(display_stream)),
                             std::istreambuf_iterator<char>());

  webdash->DisplayObjectMaker(display_object);

  display_stream.close();
}

}  // namespace o2018
