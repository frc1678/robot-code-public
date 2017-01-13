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

#ifndef AUTOGEN_AUTO_CLASSES_H_
#define AUTOGEN_AUTO_CLASSES_H_

#include <functional>
#include <map>
#include <memory>
#include <vector>

#include <lemonscript/BaseAutoFunction.h>

//<<<include>>>
#include "lemonscript-transpiler/tests/files/transpiler/unitscpp.h"


namespace AutoClass {
//<<<classes>>>

class WaitClass : public lemonscript::BaseAutoFunction {
 public:
  bool Init(std::vector<void *> ls_arg_list);
  bool Periodic(std::vector<void *> ls_arg_list);

 private:
  //<<<vars>>>
  int start_time = 0;
};
}

namespace AutoGenerator {
//<<<generators>>>
std::unique_ptr<lemonscript::BaseAutoFunction> NewWaitCommand();

std::map<std::string, std::function<std::unique_ptr<lemonscript::BaseAutoFunction>()>> GetAutoGenerators();
}

#endif
