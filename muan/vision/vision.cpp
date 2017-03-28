#include "muan/vision/vision.h"
#include <cmath>

namespace muan {

namespace vision {

int ConversionCode(VisionThresholds::ColorSpace from, VisionThresholds::ColorSpace to) {
  constexpr int conversion_matrix[3][3] = {
      {139, CV_RGB2HSV, CV_RGB2BGR}, {CV_HSV2RGB, 139, CV_HSV2BGR}, {CV_BGR2RGB, CV_BGR2HSV, 139}};

  return conversion_matrix[from][to];
}

double Vision::CalculateAngle(double x) const {
  return x * constants_.kFovX + constants_.kCameraAngleX;
}

double Vision::CalculateDistance(double y, double height_difference) const {
  double angle = y * constants_.kFovY + constants_.kCameraAngleY;
  return height_difference / std::tan(angle);
}

Vision::Vision(VisionThresholds range, VisionConstants k) {
  range_ = range;
  constants_ = k;
}

std::vector<ContourProperties> Vision::Update(cv::Mat raw, cv::Mat image_canvas) const {
  cv::Mat image;

  cv::cvtColor(raw, image, ConversionCode(VisionThresholds::Bgr, range_.space()));
  cv::inRange(image, cv::Scalar(range_.a_low(), range_.b_low(), range_.c_low()),
              cv::Scalar(range_.a_high(), range_.b_high(), range_.c_high()), image);

  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;

  cv::cvtColor(image, image_canvas, CV_GRAY2BGR);
  cv::findContours(image, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

  std::vector<std::vector<cv::Point>> hull(contours.size());
  std::vector<ContourProperties> targets;

  for (size_t i = 0; i < contours.size(); i++) {
    convexHull(cv::Mat(contours[i]), hull[i], false);
    cv::approxPolyDP(hull[i], hull[i], 2, true);

    cv::Rect bounding = cv::boundingRect(hull[i]);
    double area = cv::contourArea(contours[i]);

    // A baseline score to determine whether or not it's even a target
    double base_score = area / (image.rows * image.cols);

    if (base_score > constants_.kMinTargetArea && base_score < constants_.kMaxTargetArea) {
      double hull_area = cv::contourArea(hull[i]);
      // Fullness is the ratio of the contour's area to that of its convex hull
      double fullness = area / hull_area;

      ContourProperties target;
      target.x = (bounding.tl() + bounding.br()).x * 0.5 / image.cols - 0.5;
      // y-axis is inverted
      target.y = -((bounding.tl() + bounding.br()).y * 0.5 / image.rows - 0.5);
      target.fullness = fullness;
      target.width = (double)bounding.size().width / image.cols;
      target.height = (double)bounding.size().height / image.rows;
      targets.push_back(target);

      cv::drawContours(image_canvas, contours, i, cv::Scalar(0, 255, 0), -1);
    }
  }
  return targets;
}

void Vision::set_constants(VisionConstants constants) { constants_ = constants; }

void Vision::set_range(VisionThresholds range) { range_ = range; }

}  // namespace vision

}  // namespace muan
