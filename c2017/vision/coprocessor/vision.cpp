#include "vision.h"

namespace c2017 {
namespace vision {

void RunVision() {
  while(true) {
    muan::vision::VisionPositionProto position;
    position->set_target_found(true);
    position->set_target_id(42);
    vision_queue.WriteMessage(position);
  }
}

}
}
