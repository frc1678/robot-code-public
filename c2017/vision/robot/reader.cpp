#include "c2017/vision/robot/reader.h"
#include "c2017/queue_manager/queue_manager.h"
#include "muan/utils/threading_utils.h"
#include "third_party/aos/vision/events/udp.h"
#include "third_party/aos/common/time.h"
#include "third_party/aos/common/util/phased_loop.h"
#include "third_party/aos/linux_code/init.h"

namespace c2017 {
namespace vision {

VisionReader::VisionReader() : vision_input_queue_{QueueManager::GetInstance()->vision_input_queue()} {
  running_ = false;
}

void VisionReader::operator()() {
  aos::time::PhasedLoop phased_loop(std::chrono::milliseconds(20));
  muan::utils::SetCurrentThreadName("VisionReader");

  aos::vision::RXUdpSocket read_socket(1678);
  char buffer[1024];
  running_ = true;

  while (running_) {
    read_socket.Recv(buffer, 1024);
    VisionInputProto position;
    position->ParseFromArray(buffer, 1024);

    vision_input_queue_->WriteMessage(position);

    phased_loop.SleepUntilNext();
  }
}

void VisionReader::Stop() { running_ = false; }
}  // namespace vision
}  // namespace c2017
