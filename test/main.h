#include "WPILib.h"
#include "drivetrain.h"

class Robot : public IterativeRobot {
public:
    Drivetrain drivetrain();
    RobotDrive drive = RobotDrive(0,1);
    Joystick throttle = Joystick(1);
    Joystick wheel = Joystick(0);
    Drivetrain_Output outputs;
    
    void TeleopPeriodic();
};
