//
//  mainLemonScript.cpp
//  lemonscript
//
//  Created by Donald Pinckney on 12/24/15.
//  Copyright © 2015 Donald Pinckney. All rights reserved.
//

#include <stdio.h>
#include <iostream>
#include <vector>
#include <fstream>


#include "PlayTestsShared.h"

#include <lemonscript/lemonscript.h>
#include <lemonscript/AvailableCppCommandDeclaration.h>
#include <lemonscript/LemonScriptCompiler.h>
#include "auto_functions.h"

//using namespace lemonscript;
//using namespace lemonscript_expressions;

int main() {

    try {
        lemonscript::LemonScriptState *state = PlayTestsShared::play_tests_make_state();
//        state->setIMPORTs({"units.auto"});
        std::string fileName = "test_underscore_var.auto";
//        std::string fileName = "test_scope.auto";

        lemonscript::LemonScriptCompiler *compiler = new lemonscript::LemonScriptCompiler(fileName, state);
        
        int i = 0;
        while (true) {
            std::cout << "Iteration " << i++ << ":" << std::endl;
            
            bool isDone = compiler->PeriodicUpdate();
            if(isDone) { break; }
        }
        
        delete compiler;
        delete state;
    } catch (std::string error) {
        std::cerr << error << std::endl;
    }

    return 0;
}
