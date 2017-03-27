#ifndef C2017_VISION_COPROCESSOR_VISION_H_
#define C2017_VISION_COPROCESSOR_VISION_H_

#include <thread>
#include <chrono>
#include "c2017/vision/queue_types.h"
#include "c2017/vision/coprocessor/sender.h"
#include "muan/vision/vision.h"

namespace c2017 {
namespace vision {

constexpr double kHeightDifferenceUpper = 1.66;

void RunVision(int camera_index);

}  // namespace vision
}  // namespace c2017

#endif  // C2017_VISION_COPROCESSOR_VISION_H_
