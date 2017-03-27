#ifndef C2017_VISION_COPROCESSOR_VISION_H_
#define C2017_VISION_COPROCESSOR_VISION_H_

#include <chrono>
#include <thread>
#include <vector>
#include "c2017/vision/queue_types.h"
#include "c2017/vision/coprocessor/sender.h"
#include "muan/vision/vision.h"

namespace c2017 {
namespace vision {

constexpr double kHeightUpper = 1.66;
constexpr double kHeightLower = 1.48;

VisionInputProto CalculatePosition(std::vector<muan::vision::ContourProperties> targets,
                                   const muan::vision::Vision& vision);
void RunVision(int camera_index);

}  // namespace vision
}  // namespace c2017

#endif  // C2017_VISION_COPROCESSOR_VISION_H_
