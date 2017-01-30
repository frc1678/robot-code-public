//
//  PlayTestsShared.cpp
//  lemonscript
//
//  Created by Donald Pinckney on 5/23/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#include "PlayTestsShared.h"

#include <sstream>

#include <lemonscript/lemonscript.h>
#include <lemonscript/AvailableCppCommandDeclaration.h>
#include <lemonscript/LemonScriptState.h>
#include <lemonscript/LemonScriptCompiler.h>

#include "auto_functions.h"

using namespace lemonscript;


LemonScriptState * PlayTestsShared::play_tests_make_state() {
    
    std::vector<const AvailableCppCommandDeclaration *> decls = AvailableCppCommandDeclaration::parseCppCommands(AutoGenerator::GetAutoGenerators());
    
    LemonScriptState *state = new LemonScriptState();
    state->declareAvailableCppCommands(decls);
    
    return state;
}

void PlayTestsShared::run_at_path(std::string path, lemonscript::LemonScriptState *state) {
    try {
        lemonscript::LemonScriptCompiler *compiler = new lemonscript::LemonScriptCompiler(path, state);
        
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
}
