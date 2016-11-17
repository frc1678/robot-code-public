#include "third_party/opencv/include/opencv2/opencv.hpp"
#include <iostream>
#include <vector>
#include <chrono>
#include <cmath>
#include <thread>

// Whether to output to a file (1) or to a window (0)
#define VIDEO_OUTPUT 1

double CalculateSkew(std::vector<cv::Point> contour,
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
  if (top.x == 0 || bottom.x == 0) {
    return std::atan(1) * 2; // pi / 2
  } else {
    double slope = ((double) top.y / top.x + bottom.y / bottom.x) / 2;
    return std::atan(slope);
  }
}

int main() {
  // Read from captured.avi
  cv::VideoCapture cap;
  cap.open("captured.avi");

#if VIDEO_OUTPUT
  cv::VideoWriter output{"output.avi", CV_FOURCC('M', 'J', 'P', 'G'), 30,
                         cv::Size(640 * 2, 480)};
#else
  cv::namedWindow("vision", cv::WINDOW_AUTOSIZE);
#endif

  cv::Mat image, image_canvas, raw;

  auto last_time = std::chrono::steady_clock::now();

  // The last position at which a goal was found
  cv::Point2f last_pos;

  while (cap.isOpened()) {
    last_time = std::chrono::steady_clock::now();

    cap >> raw;

    // If we hit the end of the video; stop
    if (raw.empty()) {
      //break;
    }

    cv::cvtColor(raw, image, CV_BGR2HSV);

    // Threshold it to find green blobs
    cv::inRange(image, cv::Scalar(50, 0, 60), cv::Scalar(100, 255, 255), image);

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
      double base_score = std::log(area) / (.1 + std::pow(fullness - .2, 2));

      if (base_score > 20) {
        // It passed the baseline tests, it might be a target
        targets.push_back(i);

        std::vector<cv::Point> skewbox;
        double skew = CalculateSkew(contours[i], skewbox);

        // Penalize based on the distance from the last target found
        double distance = cv::norm(bounding.center - last_pos);
        // Don't limit it too much if it's really far away
        double distance_penalty = std::min(distance, 30.0) * .08;

        // Formula subject to tuning
        double target_score = base_score / (1 + skew) - distance_penalty;

        if (target_score > best_score) {
          best_target = i;
          best_score = target_score;
          best_pos = bounding.center;
        }
      }
    }

    for (auto target : targets) {
      cv::Scalar color(0, 0, 255);
      if (target == best_target) {
        color = cv::Scalar(0, 255, 0);
        last_pos = best_pos;
      }

      cv::drawContours(image_canvas, contours, target, color, 8);
      cv::drawContours(image_canvas, hull, target, cv::Scalar(255, 0, 0), 5);
    }

#if VIDEO_OUTPUT
    cv::Mat splitscreen(image_canvas.rows, image_canvas.cols * 2, CV_8UC3);
    image_canvas.copyTo(
        splitscreen(cv::Rect(0, 0, image_canvas.cols, image_canvas.rows)));
    raw.copyTo(splitscreen(
        cv::Rect(image_canvas.cols, 0, image_canvas.cols, image_canvas.rows)));
    output.write(splitscreen);
#else
    cv::resize(image_canvas, image_canvas,
               cv::Size(image_canvas.cols / 3, image_canvas.rows / 3));
    cv::resize(raw, raw, cv::Size(raw.cols / 3, raw.rows / 3));
    cv::imshow("vision", image_canvas);
    cv::imshow("raw", raw);
    cv::waitKey(1);
#endif

    auto count = std::chrono::duration_cast<std::chrono::milliseconds>(
                     std::chrono::steady_clock::now() - last_time).count();

    std::cout << count << std::endl;
  }
}
