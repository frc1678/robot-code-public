#include <WPILib.h>
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

class MotorCalibration : public IterativeRobot {
 public:
  MotorCalibration() {
    int port_to_be_calibrated;

    std::cout << "Which port do you want to be calibrated?" << std::endl;
    std::cin >> port_to_be_calibrated;

    motor_ = std::make_unique<VictorSP>(port_to_be_calibrated);
    controller_ = std::make_unique<GenericHID>(0);
  }

  void TeleopPeriodic() override {
    if (controller_->GetRawButton(0 /* This might need to be 1 */)) {
      motor_->Set(1.0);
      std::this_thread::sleep_for(std::chrono::seconds(2));
      motor_->Set(-1.0);
      std::this_thread::sleep_for(std::chrono::seconds(2));
    }
  }

 private:
  VictorSP* motor_;
  GenericHID controller_;
};

START_ROBOT_CLASS(MotorCalibration);
