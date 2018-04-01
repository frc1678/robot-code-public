#ifndef MUAN_VISION_VISION_H_
#define MUAN_VISION_VISION_H_

#include <opencv2/opencv.hpp>
#include <cmath>
#include <memory>
#include <vector>
#include "muan/vision/config.pb.h"

namespace muan {
namespace vision {

// Return the OpenCV conversion code for converting from `from` to `to`.
int ConversionCode(VisionThresholds::ColorSpace from,
                   VisionThresholds::ColorSpace to);

struct ContourProperties {
  double x;         // -0.5 to 0.5. 0.5 is right of image.
  double y;         //  -0.5 to 0.5. 0.5 is top of image.
  double fullness;  // Area of target / area of bounding rect
  double width;     // How much of the image's width it occupies, from 0 to 1
  double height;    // How much of the image's height it occupies, from 0 to 1
};

struct VisionConstants {
  double kFovX;  // Horizontal field of view, in radians
  double kFovY;  // Vertical field of view, in radians
  double kCameraAngleX;  // Angle of camera is to the left of the robot, radians
  double kCameraAngleY;  // Angle of the camera above horizontal, in radians
  double kMinTargetArea;  // Minumum area of a target compared with image area
  double kMaxTargetArea;  // Maximum area of a target compared with image area
};

class Vision {
 public:
  Vision(VisionThresholds range, VisionConstants k);

  // Get all potential targets within size limits
  std::vector<ContourProperties> Update(cv::Mat raw,
                                        cv::Mat image_canvas) const;

  // x: -0.5 is left of image, 0.5 is right
  // Returns angle of robot to the left of the target, in radians
  double CalculateAngle(double x) const;

  // y: -0.5 is bottom of image, 0.5 is top
  // height_difference: Height of goal above camera, in meters
  // Returns distance in meters
  double CalculateDistance(double y, double height_difference) const;

  void set_constants(VisionConstants constants);
  void set_range(VisionThresholds range);

 protected:
  // Robot constants relavent to vision
  VisionConstants constants_;
  // Last place the goal was
  VisionThresholds range_;
};

}  // namespace vision
}  // namespace muan

#endif  // MUAN_VISION_VISION_H_
