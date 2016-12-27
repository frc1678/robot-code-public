#ifndef MUAN_VISION_VISION_H_
#define MUAN_VISION_VISION_H_

#include <cmath>
#include <vector>
#include "third_party/opencv/include/opencv2/opencv.hpp"

namespace muan {

class VisionScorer {
 public:
  virtual double GetScore(double distance_to_target,
                          double distance_from_previous,
                          double skew,
                          double width,
                          double height,
                          double fullness) = 0;
};

class Vision {
 public:
  struct VisionConstants {
    double kFovX; // Horizontal field of view, in radians
    double kFovY; // Vertical field of view, in radians
    double kCameraAngle; // Angle of the camera above horizontal, in radians
    double kHeightDifference; // Height of goal above camera, in meters
    double kFullness; // area of target / area of bounding rect
  };

  struct VisionStatus {
    bool target_exists;
    double distance_to_target; // Distance to target, in meters
    double angle_to_target; // Angle to target (too far to right is positive), in radians
    cv::Mat image_canvas; // Display of what exactly the robot thinks it's seeing
  };

  Vision(cv::Scalar lower_bound,
         cv::Scalar upper_bound,
         VisionScorer* scorer,
         VisionConstants k);
  VisionStatus Update(cv::Mat raw);

 protected:
  double CalculateDistance(std::vector<cv::Point> points, int rows);
  double CalculateSkew(std::vector<cv::Point> contour,
                       std::vector<cv::Point>& out);
  // Lower end of color range
  cv::Scalar lower_bound_;
  // Upper end of color range
  cv::Scalar upper_bound_;
  // The formula to score the targets
  VisionScorer* scorer_;
  // Robot constants relavent to vision
  VisionConstants constants_;
  // Last place the goal was
  cv::Point2f last_pos_;
};

}
#endif // MUAN_VISION_VISION_H_
