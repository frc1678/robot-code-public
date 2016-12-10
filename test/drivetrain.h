#ifndef TEST_DRIVETRAIN_H
#define TEST_DRIVETRAIN_H

class DriveTrain {
 public:
  std::tuple<double,double> Update(double wheel, double joystick,bool quickturn);
};

#endif
