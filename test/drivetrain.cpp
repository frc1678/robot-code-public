#include "drivetrain.h"


test1::Drivetrain_Output test1::Drivetrain::Update(double joystick, double wheel, bool quick_turn) {
        
        if (quick_turn) {
            output.left_voltage = -wheel;
            output.right_voltage = wheel;
            return output;
        } else {
            output.left_voltage = (joystick + wheel) / 2;
            output.right_voltage = (joystick - wheel) / 2;
            return output;
        }
    }
    

