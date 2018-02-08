#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include "WPILib.h"
#include "muan/teleop/button.h"
#include "muan/teleop/joystick.h"

class MotorCalibration : public IterativeRobot {
 public:
  muan::teleop::Joystick* button_maker_;
  muan::teleop::Button* calibrate_start_ =
      button_maker_->MakePov(0, muan::teleop::Pov::kWest);
  int port_to_be_calibrated;

  MotorCalibration() {
    std::cout << "Which port do you want to be calibrated?" << std::endl;
    std::cin >> port_to_be_calibrated;
    motor_ = std::make_unique<VictorSP>(port_to_be_calibrated);
  }

  void TeleopPeriodic() override {
    button_maker_->Update();
    if (calibrate_start_->is_pressed()) {
      motor_->Set(1.0);
      std::this_thread::sleep_for(std::chrono::seconds(2));
      motor_->Set(-1.0);
      std::this_thread::sleep_for(std::chrono::seconds(2));
    }
  }

 private:
  std::unique_ptr<VictorSP> motor_;
};

START_ROBOT_CLASS(MotorCalibration);
