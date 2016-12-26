#ifndef test1_H_
#define test1_H_

namespace test1 {
    
struct Drivetrain_Output {
    double left_voltage;
    double right_voltage;
};
    
    Drivetrain_Output output;
    
class Drivetrain {
    public:
        Drivetrain_Output Update(double joystick, double wheel, bool quick_turn);
    };
    
}
#endif //TEST1_H_