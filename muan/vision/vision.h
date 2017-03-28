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
int ConversionCode(VisionThresholds::ColorSpace from, VisionThresholds::ColorSpace to);

class VisionScorer {
 public:
  virtual double GetScore(double distance_to_target,      // in meters
                          double distance_from_previous,  // in pixels
                          double skew,                    // of bounding box, in radians
                          double width,                   // in pixels
                          double height,                  // in pixels
                          double fullness) = 0;           // area / bounding box area

  virtual void Morph(cv::Mat) {}  // Erode and dilate go in here. By default does nothing.
};

class Vision {
 public:
  struct VisionConstants {
    double kFovX;              // Horizontal field of view, in radians
    double kFovY;              // Vertical field of view, in radians
    double kCameraAngleX;      // Angle of the camera to the left of the robot, radians
    double kCameraAngleY;      // Angle of the camera above horizontal, in radians
    double kHeightDifference;  // Height of goal above camera, in meters
    double kFullness;          // area of target / area of bounding rect

    double kMinTargetArea;  // Minumum area of a target compared with image area
    double kMaxTargetArea;
  };

  struct VisionStatus {
    bool target_exists;
    double distance_to_target;  // Distance to target, in meters
    double angle_to_target;     // Angle to target (too far to right is positive), in radians
    cv::Mat image_canvas;       // Display of what exactly the robot thinks it's seeing
  };

  Vision(VisionThresholds range, std::shared_ptr<VisionScorer> scorer, VisionConstants k);
  VisionStatus Update(cv::Mat raw);

  void set_constants(VisionConstants constants);
  void set_range(VisionThresholds range);

 protected:
  double CalculateDistance(std::vector<cv::Point> points, int rows);
  double CalculateSkew(std::vector<cv::Point> contour, std::vector<cv::Point>* out);
  // The formula to score the targets
  std::shared_ptr<VisionScorer> scorer_;
  // Robot constants relavent to vision
  VisionConstants constants_;
  // Last place the goal was
  cv::Point2f last_pos_;
  VisionThresholds range_;
};

}  // namespace vision

}  // namespace muan

#endif  // MUAN_VISION_VISION_H_
