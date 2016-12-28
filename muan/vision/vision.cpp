#include "muan/vision/vision.h"
#include <cmath>

namespace muan {

double Vision::CalculateDistance(std::vector<cv::Point> points, int rows) {
  // Average together height of each point
  double angle = 0;
  for (auto& p : points) {
    angle += p.y;
  }
  angle /= points.size();
  // Scale angle from -fov/2 to fov/2
  angle = (angle / rows - 0.5) * constants_.kFovY;

  double distance = constants_.kHeightDifference /
                    std::tan(angle + constants_.kCameraAngle);
  return distance;
}

double Vision::CalculateSkew(std::vector<cv::Point> contour,
                             std::vector<cv::Point>& out) {
  out.resize(4);
  std::vector<cv::Point>& quad = out;
  // Find extrema for x + y and x - y to find the corners of the goal
  for (auto& p : contour) {
    if (p.x + p.y > quad[0].x + quad[0].y) {
      quad[0] = p;
    }
    if (p.x - p.y > quad[1].x - quad[1].y) {
      quad[1] = p;
    }
    if (-p.x - p.y > -quad[2].x - quad[2].y) {
      quad[2] = p;
    }
    if (-p.x + p.y > -quad[3].x + quad[3].y) {
      quad[3] = p;
    }
  }

  // Find the two vectors representing the top and bottom of the goal
  auto top = quad[0] - quad[3];
  auto bottom = quad[1] - quad[2];

  // Find their slopes and average the values together to get a goal angle
  return (std::atan2(top.y, top.x) + std::atan2(bottom.y, bottom.x)) / 2;
}

Vision::Vision(ColorRange range, VisionScorer* scorer, VisionConstants k) {
  range_ = range;
  scorer_ = scorer;
  constants_ = k;
  last_pos_  = cv::Point2f();
  // It has to be set to something, right?
  last_pos_.x = 0;
  last_pos_.y = 0;
}

Vision::VisionStatus Vision::Update(cv::Mat raw) {
  cv::Mat image_canvas, image;
  VisionStatus retval;
  retval.target_exists = false;
  retval.distance_to_target = 0;
  retval.angle_to_target = 0;

  cv::cvtColor(raw, image, range_.colorspace);
  cv::inRange(image, range_.lower_bound, range_.upper_bound, image);

  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;

  cv::cvtColor(image, image_canvas, CV_GRAY2BGR);
  cv::findContours(image, contours, hierarchy, CV_RETR_TREE,
                   CV_CHAIN_APPROX_SIMPLE);

  std::vector<std::vector<cv::Point>> hull(contours.size());
  std::vector<size_t> targets;

  double best_score = -1;
  size_t best_target;
  cv::Point best_pos;

  for (size_t i = 0; i < contours.size(); i++) {
    convexHull(cv::Mat(contours[i]), hull[i], false);
    cv::approxPolyDP(hull[i], hull[i], 2, true);
    cv::Scalar color(0, 255, 0);

    cv::RotatedRect bounding = cv::minAreaRect(hull[i]);

    double area = cv::contourArea(contours[i]);
    // Fullness is the ratio of the contour's area to that of its convex hull
    double fullness = area / cv::contourArea(hull[i]);

    // A baseline score to determine whether or not it's even a target
    double base_score = std::log(area) / (.1 + std::pow(fullness - constants_.kFullness, 2));

    // originally 20, wtf went wrong?
    if (base_score > -1) {
      // It passed the baseline tests, it might be a target
      targets.push_back(i);

      std::vector<cv::Point> skewbox;

      double skew = CalculateSkew(contours[i], skewbox);
      double distance_from_previous = cv::norm(bounding.center - last_pos_);
      double distance_to_target = CalculateDistance(contours[i], image.rows);
      double width = (skewbox[0] + skewbox[1] - skewbox[2] - skewbox[3]).x / 2;
      double height = (skewbox[0] - skewbox[1] - skewbox[2] + skewbox[3]).y / 2;

      // Formula subject to tuning
      double target_score = scorer_->GetScore(distance_to_target,
                                              distance_from_previous,
                                              skew,
                                              width,
                                              height,
                                              fullness);

      if (target_score > best_score) {
        best_target = i;
        best_score = target_score;
        best_pos = bounding.center;

        retval.target_exists = true;
        retval.distance_to_target = distance_to_target;
        // average together points
        double angle_to_target = (skewbox[0] + skewbox[1] + skewbox[2] + skewbox[3]).x / 4;
        angle_to_target = (angle_to_target / image.cols - 0.5) * constants_.kFovX;
        retval.angle_to_target = angle_to_target;
      }
    }
  }

  for (auto target : targets) {
    cv::Scalar color(0, 0, 255);
    if (target == best_target) {
      color = cv::Scalar(0, 255, 0);
      last_pos_ = best_pos;
    }

    cv::drawContours(image_canvas, contours, target, color, 8);
    cv::drawContours(image_canvas, hull, target, cv::Scalar(255, 0, 0), 5);
  }
  retval.image_canvas = image_canvas;
  return retval;
}

}
