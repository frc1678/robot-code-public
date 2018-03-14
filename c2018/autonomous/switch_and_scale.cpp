#include "c2018/autonomous/switch_and_scale.h"

#include <cmath>

namespace c2018 {
namespace autonomous {

void SwitchAndScale::LeftSwitchLeftScale() {
  // LL - Switch is left, scale is left
  LOG(INFO, "Running LEFT SWITCH LEFT SCALE auto");

  // Drive to backside of switch
  StartDrivePath(3.2, -3, 0.0, -1);
  WaitUntilDriveComplete();
  StartDrivePath(3.2, -5, M_PI * 0.25, -1);
  WaitUntilDriveComplete();

  // Score here

  // Drive to cubes in PZ
  StartDrivePath(2., kCubeXFromReverseWall, 0.0, 1);
  WaitUntilDriveComplete();

  // Pick up cube here

  // Drive to scale
  StartDrivePath(2.5, kScaleXFromReverseWall, 0.0, -1);
  WaitUntilDriveComplete();

  // Drive to 2nd cube
  StartDrivePath(1.75, kCubeXFromReverseWall, 0.0, 1);
  WaitUntilDriveComplete();

  // Drive back to scale
  StartDrivePath(2.5, kScaleXFromReverseWall, 0.0, -1);
  WaitUntilDriveComplete();
}

void SwitchAndScale::RightSwitchRightScale() {
  // RR - Switch is right, scale is right
  LOG(INFO, "Running RIGHT SWITCH RIGHT SCALE auto");

  // Initial drive toward backside of switch
  StartDrivePath(-2.5, -2.5, 0.0, -1);
  WaitUntilDriveComplete();
  StartDrivePath(-2., -5., M_PI * 0.25, -1);
  WaitUntilDriveComplete();

  // Score cube in switch here

  // Pick up next cube
  StartDrivePath(-1.25, kCubeXFromReverseWall, 0.0, 1);
  WaitUntilDriveComplete();

  // Drive to scale
  StartDrivePath(-1.75, kScaleXFromReverseWall, 0.0, -1);
  WaitUntilDriveComplete();

  // Score on scale here

  // Drive to 2nd cube
  StartDrivePath(-0.8, kCubeXFromReverseWall, 0.0, 1);
  WaitUntilDriveComplete();

  // Intake 2nd cube here

  // Drive to scale
  StartDrivePath(-1.75, c2018::autonomous::kScaleXFromReverseWall, 0.0, -1);
  WaitUntilDriveComplete();

  // Score here
}

void SwitchAndScale::RightSwitchLeftScale() {
  // RL - Switch is right, scale is left
  LOG(INFO, "Running RIGHT SWITCH LEFT SCALE auto");

  // Initial drive to backside of switch
  StartDrivePath(-2.5, -2.5, 0.0, -1);
  WaitUntilDriveComplete();
  StartDrivePath(-2, -5.5, M_PI * -0.25, -1);
  WaitUntilDriveComplete();

  // Score here

  // Drive to next cube
  StartDrivePath(-1.5, kCubeXFromReverseWall, 0.0, 1);
  WaitUntilDriveComplete();

  // Intake cube here

  // Drive to scale
  StartDrivePath(2.0, -5.5, M_PI * 0.5, -1);
  WaitUntilDriveComplete();
  StartDrivePath(2.5, kScaleXFromReverseWall, 0.0, -1);
  WaitUntilDriveComplete();

  // Score on scale here

  // Drive to 2nd cube
  StartDrivePath(2.25, kCubeXFromReverseWall, 0.0, 1);
  WaitUntilDriveComplete();

  // Intake 2nd cube here

  // Drive back to scale
  StartDrivePath(2.5, kScaleXFromReverseWall, 0.0, -1);
  WaitUntilDriveComplete();

  // Score on scale here
}

void SwitchAndScale::LeftSwitchRightScale() {
  // Switch is left, scale is right
  LOG(INFO, "Running LEFT SWITCH RIGHT SCALE auto");

  // Drive to backside of switch
  StartDrivePath(3.2, -3, 0.0, -1);
  WaitUntilDriveComplete();
  StartDrivePath(3.2, -5, M_PI * 0.25, -1);
  WaitUntilDriveComplete();

  // Score here

  // Drive to cubes in PZ
  StartDrivePath(2., kCubeXFromReverseWall, 0.0, 1);
  WaitUntilDriveComplete();

  // Pick up cube here

  // Drive to scale on right side
  StartDrivePath(-1.0, -5.75, M_PI * -0.5, -1);
  WaitUntilDriveComplete();
  StartDrivePath(-1.2, kScaleXFromReverseWall, 0.0, -1);
  WaitUntilDriveComplete();

  // Score on scale here

  // Drive to 2nd cube
  StartDrivePath(-1.25, kCubeXFromReverseWall, 0.0, 1);
  WaitUntilDriveComplete();

  // Intake cube here

  // Drive to scale again
  StartDrivePath(-1.2, kScaleXFromReverseWall, 0.0, -1);
  WaitUntilDriveComplete();

  // Score on scale here
}

}  // namespace autonomous
}  // namespace c2018
