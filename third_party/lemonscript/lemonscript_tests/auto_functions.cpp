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


#include "./auto_functions.h"

//// vim: set filetype=cpp:

//TODO(Wesley) Don't do this every tick
//TODO(Wesley or Kyle) Clean up this code (remove structs)
namespace ConvertArgs {
    // These operators are in structs because C++ doesn't support
    // partial function overloading, which is needed to specialize
    // on Units<U1, U2, U3, U4>
    template<typename T>
    struct Convert {
        T operator()(void* arg) {
            return *static_cast<T*>(arg);
        }
    };
    
    template<typename T>
    struct Convert<T*>{
        T* operator()(void* arg) {
            return static_cast<T*>(arg);
        }
    };
}




namespace AutoClass {
    //<<<classes>>>
    
    bool PrintIntClass::Init(std::vector<void *> ls_arg_list) {
        // BEGIN AUTO GENERATED CODE
        int v = ConvertArgs::Convert<int>()(ls_arg_list[0+1]);
        // END AUTO GENERATED CODE
        
        return false;
    }
    
    bool PrintIntClass::Periodic(std::vector<void *> ls_arg_list) {
        // BEGIN AUTO GENERATED CODE
        int v = ConvertArgs::Convert<int>()(ls_arg_list[0+1]);
        // END AUTO GENERATED CODE
        
        printf("%d\n", v);
        return true;
    }
    
    
    bool PrintFloatClass::Init(std::vector<void *> ls_arg_list) {
        // BEGIN AUTO GENERATED CODE
        float v = ConvertArgs::Convert<float>()(ls_arg_list[0+1]);
        // END AUTO GENERATED CODE
        
        return false;
    }
    
    bool PrintFloatClass::Periodic(std::vector<void *> ls_arg_list) {
        // BEGIN AUTO GENERATED CODE
        float v = ConvertArgs::Convert<float>()(ls_arg_list[0+1]);
        // END AUTO GENERATED CODE
        
        printf("%f\n", v);
        return true;
    }
    
    
    bool PrintBoolClass::Init(std::vector<void *> ls_arg_list) {
        // BEGIN AUTO GENERATED CODE
        bool v = ConvertArgs::Convert<bool>()(ls_arg_list[0+1]);
        // END AUTO GENERATED CODE
        
        return false;
    }
    
    bool PrintBoolClass::Periodic(std::vector<void *> ls_arg_list) {
        // BEGIN AUTO GENERATED CODE
        bool v = ConvertArgs::Convert<bool>()(ls_arg_list[0+1]);
        // END AUTO GENERATED CODE
        
        printf("%s\n", v ? "true" : "false");
        return true;
    }
    
    
    bool PrintAllClass::Init(std::vector<void *> ls_arg_list) {
        // BEGIN AUTO GENERATED CODE
        int theInt = ConvertArgs::Convert<int>()(ls_arg_list[0+1]);
        float theFloat = ConvertArgs::Convert<float>()(ls_arg_list[1+1]);
        bool theBool = ConvertArgs::Convert<bool>()(ls_arg_list[2+1]);
        // END AUTO GENERATED CODE
        
        return false;
    }
    
    bool PrintAllClass::Periodic(std::vector<void *> ls_arg_list) {
        // BEGIN AUTO GENERATED CODE
        int theInt = ConvertArgs::Convert<int>()(ls_arg_list[0+1]);
        float theFloat = ConvertArgs::Convert<float>()(ls_arg_list[1+1]);
        bool theBool = ConvertArgs::Convert<bool>()(ls_arg_list[2+1]);
        // END AUTO GENERATED CODE
        
        printf("(%d, %f, %s)\n", theInt, theFloat, theBool ? "true" : "false");
        return true;
    }
    
    
    bool DoNothingClass::Init(std::vector<void *> /* ls_arg_list */) {
        // BEGIN AUTO GENERATED CODE
        
        // END AUTO GENERATED CODE
        
        printf("Do nothing INIT\n");
        return false;
    }
    
    bool DoNothingClass::Periodic(std::vector<void *> /* ls_arg_list */) {
        // BEGIN AUTO GENERATED CODE
        
        // END AUTO GENERATED CODE
        
        printf("Do nothing PERIODIC\n");
        return true;
    }
    
}

namespace AutoGenerator {
    //<<<generators>>>
    std::unique_ptr<BaseAutoFunction> NewPrintIntCommand() {
        return std::make_unique<AutoClass::PrintIntClass>();
    }
    
    std::unique_ptr<BaseAutoFunction> NewPrintFloatCommand() {
        return std::make_unique<AutoClass::PrintFloatClass>();
    }
    
    std::unique_ptr<BaseAutoFunction> NewPrintBoolCommand() {
        return std::make_unique<AutoClass::PrintBoolClass>();
    }
    
    std::unique_ptr<BaseAutoFunction> NewPrintAllCommand() {
        return std::make_unique<AutoClass::PrintAllClass>();
    }
    
    std::unique_ptr<BaseAutoFunction> NewDoNothingCommand() {
        return std::make_unique<AutoClass::DoNothingClass>();
    }
    
    
    std::map<std::string, std::function<std::unique_ptr<BaseAutoFunction>()>> GetAutoGenerators() {
        std::map<std::string, std::function<std::unique_ptr<BaseAutoFunction>()>> return_map;
        
        //<<<addgenerators>>>
        std::function<std::unique_ptr<BaseAutoFunction>()> NewPrintIntGenerator = NewPrintIntCommand;
        return_map["PrintInt-int"] = NewPrintIntCommand;
        
        std::function<std::unique_ptr<BaseAutoFunction>()> NewPrintFloatGenerator = NewPrintFloatCommand;
        return_map["PrintFloat-float"] = NewPrintFloatCommand;
        
        std::function<std::unique_ptr<BaseAutoFunction>()> NewPrintBoolGenerator = NewPrintBoolCommand;
        return_map["PrintBool-bool"] = NewPrintBoolCommand;
        
        std::function<std::unique_ptr<BaseAutoFunction>()> NewPrintAllGenerator = NewPrintAllCommand;
        return_map["PrintAll-int-float-bool"] = NewPrintAllCommand;
        
        std::function<std::unique_ptr<BaseAutoFunction>()> NewDoNothingGenerator = NewDoNothingCommand;
        return_map["DoNothing-"] = NewDoNothingCommand;
        
        
        return return_map;
        
    }
}
