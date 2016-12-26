#include "main.h"

Robot::TeleopPeriodic() {
    outputs = drivetrain.Update(throttle.GetY();, wheel.GetX();, wheel.GetRawButton(0);)
    drive.TankDrive(outputs.left_voltage, outputs.right_voltage)
}

