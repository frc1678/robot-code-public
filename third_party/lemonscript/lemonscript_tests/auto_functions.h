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

#include <vector>
#include <functional>
#include <memory>
#include <map>
#include "BaseAutoFunction.h"
#include <string>

using lemonscript::BaseAutoFunction;

//<<<include>>>
#include <stdio.h>
#include <stdio.h>
#include <stdio.h>
#include <stdio.h>
#include <stdio.h>



namespace AutoClass {
    //<<<classes>>>
    
    class PrintIntClass : public BaseAutoFunction {
        public:
        bool Init(std::vector<void *> ls_arg_list);
        bool Periodic(std::vector<void *> ls_arg_list);
        private:
        //<<<vars>>>
    };
    
    
    class PrintFloatClass : public BaseAutoFunction {
        public:
        bool Init(std::vector<void *> ls_arg_list);
        bool Periodic(std::vector<void *> ls_arg_list);
        private:
        //<<<vars>>>
    };
    
    
    class PrintBoolClass : public BaseAutoFunction {
        public:
        bool Init(std::vector<void *> ls_arg_list);
        bool Periodic(std::vector<void *> ls_arg_list);
        private:
        //<<<vars>>>
    };
    
    
    class PrintAllClass : public BaseAutoFunction {
        public:
        bool Init(std::vector<void *> ls_arg_list);
        bool Periodic(std::vector<void *> ls_arg_list);
        private:
        //<<<vars>>>
    };
    
    
    class DoNothingClass : public BaseAutoFunction {
        public:
        bool Init(std::vector<void *> ls_arg_list);
        bool Periodic(std::vector<void *> ls_arg_list);
        private:
        //<<<vars>>>
    };
    
}

namespace AutoGenerator {
    //<<<generators>>>
    std::unique_ptr<BaseAutoFunction> NewPrintIntCommand();
    
    std::unique_ptr<BaseAutoFunction> NewPrintFloatCommand();
    
    std::unique_ptr<BaseAutoFunction> NewPrintBoolCommand();
    
    std::unique_ptr<BaseAutoFunction> NewPrintAllCommand();
    
    std::unique_ptr<BaseAutoFunction> NewDoNothingCommand();
    
    
    std::map<std::string, std::function<std::unique_ptr<BaseAutoFunction>()>> GetAutoGenerators();
}

#endif
