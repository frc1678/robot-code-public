# Vision Code Documentation

### `muan::VisionScorer`
**A pure virtual base class for determining which target out of several options
is what the robot should be aiming at.**

```c++
virtual double GetScore(double distance_to_target,
                        double distance_from_previous,
                        double skew,
                        double width,
                        double height,
                        double fullness) = 0;
```
Override this method to determine what the robot should be aiming at based on a
number of factors:

* The distance, in meters, from the robot to the target
* The distance, in pixels, from the previous best target to the current option
* The skew, or angle that the bounding rectangle is tilted, in radians
* The width of the target, in pixels
* The height of the target, in pixels
* The fullness, or area of the target divided by area of the bounding rectangle

Some of these parameters may be used or not used depending on what the target is. For
an example see `muan/vision/example/vision_example.cpp`. A value of -1 or less means
that it cannot be a target.

### `muan::Vision`
**A class that tracks a target based on values passed to the constructor.**

This is a struct to hold constants relavent to vision:
```c++
struct muan::Vision::VisionConstants {
  double kFovX; // Horizontal field of view, in radians
  double kFovY; // Vertical field of view, in radians
  double kCameraAngle; // Angle of the camera above horizontal, in radians
  double kHeightDifference; // Height of goal above camera, in meters
  double kFullness; // area of target / area of bounding rect
};
```

This is a struct to hold the output of vision:
```c++
struct muan::Vision::VisionStatus {
  bool target_exists; // Whether a decent target exists in the image
  double distance_to_target; // Distance to target, in meters
  double angle_to_target; // Angle to target (too far to right is positive), in radians
  cv::Mat image_canvas; // Display of what exactly the robot thinks it's seeing
};
```

This is a struct to hold the information needed to threshold an image:
```c++
struct muan::Vision::ColorRange {
  cv::Scalar lower_bound; // Lower end of color range
  cv::Scalar upper_bound; // Upper end of color range
  // Colorspace used for in_range (such as CV_BGR2YUV for
  // the YUV colorspace).
  int colorspace;
};
```

The constructor to Vision:
```c++
Vision(ColorRange range, std::shared_ptr<VisionScorer> scorer, VisionConstants k);
```

Detecting the target:
```c++
VisionStatus Update(cv::Mat raw);
```
Given the raw image, find any targets based on the scorer passed to the constructor.
Returns whether the target exists, the distance and angle to the target, and the
processed image in the form of a VisionStatus struct. First, it thresholds the image based
on the color range and colorspace it was given. It then finds all contours. The contours
that are less that .2% of the image are treated as noise and are ignored immediately. For
the remaining ones, the skew, fullness, and distances are calculated and passed to the
VisionScorer. The contour with the highest score is selected if the score is more than -1.

### What are colorspaces?

A colorspace is a way of storing a color as three numbers. You may be familiar
with RGB (Red-Green-Blue) as a way of storing a color. Ideally the colorspace
you choose will have well-defined upper and lower values for the target for all
three numbers. If all colorspaces work similarly well, RGB should be used since it
is the most intuitive.
