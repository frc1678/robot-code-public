/***********************************************\
*  _____          _   _  _____ ______ _____  _  *
* |  __ \   /\   | \ | |/ ____|  ____|  __ \| | *
* | |  | | /  \  |  \| | |  __| |__  | |__) | | *
* | |  | |/ /\ \ | . ` | | |_ |  __| |  _  /| | *
* | |__| / ____ \| |\  | |__| | |____| | \ \|_| *
* |_____/_/    \_\_| \_|\_____|______|_|  \_(_) *
*                                               *
* This file was automatically generated - any   *
* changes that you make to it *will* be deleted *
* in the future. Do not change this file,       *
* instead change the .func file for the auto    *
* function that you want to change. If you have *
* any questions, or there is a bug in the       *
* transpiler, talk to Wesley about it.          *
\***********************************************/

#include "auto_functions.h"

#include <iostream>

//// vim: set filetype=cpp:

//TODO(Wesley) Don't do this every tick
namespace ConvertArgs {
int ls_convert_int(void *arg) {
  return *((int *)arg);
}
bool ls_convert_bool(void *arg) {
  return *((bool *)arg);
}
float ls_convert_float(void *arg) {
  return *((float *)arg);
}
std::string ls_convert_string(void *arg) {
  return *((std::string *)arg);
}
Time ls_convert_time(void *arg) {
  return ls_convert_float(arg) * s;
}
Distance ls_convert_distance(void *arg) {
  return ls_convert_float(arg) * m;
}
Angle ls_convert_angle(void *arg) {
  return ls_convert_float(arg) * rad;
}
Velocity ls_convert_velocity(void *arg) {
  return ls_convert_float(arg) * (m / s);
}
Acceleration ls_convert_acceleration(void *arg) {
  return ls_convert_float(arg) * (m / (s * s));
}
AngularVelocity ls_convert_angularvelocity(void *arg) {
  return ls_convert_float(arg) * (rad / s);
}
Voltage ls_convert_voltage(void *arg) {
  return ls_convert_float(arg) * V;
}
}

namespace AutoClass {
//<<<classes>>>

bool WaitClass::Init(std::vector<void *> ls_arg_list) {
  // BEGIN AUTO GENERATED CODE
//    Time time = ConvertArgs::ls_convert_time(ls_arg_list[1]);
    int time = ConvertArgs::ls_convert_int(ls_arg_list[1]);

  // END AUTO GENERATED CODE

  start_time = 0;
  return false;
}

bool WaitClass::Periodic(std::vector<void *> ls_arg_list) {
  // BEGIN AUTO GENERATED CODE
//  Time time = ConvertArgs::ls_convert_time(ls_arg_list[1]);
    int time = ConvertArgs::ls_convert_int(ls_arg_list[1]);
  // END AUTO GENERATED CODE
    start_time++;
    
    if(start_time >= time) {
        std::cout << "Done waiting for " << time << " seconds." << std::endl;
        return true;
    }
    
    return false;
}
}

namespace AutoGenerator {
//<<<generators>>>
    
std::unique_ptr<lemonscript::BaseAutoFunction> NewWaitCommand() {
    return std::make_unique<AutoClass::WaitClass>();
}

std::map<std::string, std::function<std::unique_ptr<lemonscript::BaseAutoFunction>()>> GetAutoGenerators() {
  std::map<std::string, std::function<std::unique_ptr<lemonscript::BaseAutoFunction>()>> return_map;

  //<<<addgenerators>>>
  std::function<std::unique_ptr<lemonscript::BaseAutoFunction>()> NewWaitGenerator = NewWaitCommand;
  return_map["Wait-int"] = NewWaitCommand;

  return return_map;
}
}
