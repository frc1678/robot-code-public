//
//  LemonScriptCompiler.hpp
//  FiniteStateMachine
//
//  Created by Donald Pinckney on 12/30/15.
//  Copyright © 2015 Donald Pinckney. All rights reserved.
//

#ifndef LemonScriptCompiler_hpp
#define LemonScriptCompiler_hpp

#include <stdio.h>
#include <sstream>
#include <vector>
#include <iostream>
#include <fstream>

#include "LemonScriptTokenizer.h"
#include "AvailableCppCommandDeclaration.h"
#include "LemonScriptState.h"
#include "Commands/SequentialCommand.h"
#include "ParsingUtils.h"

#include "lemonscript.h"

class lemonscript::LemonScriptCompiler {
    
    LemonScriptState *state;
    SequentialCommand *rootSequence;
    bool isDone = false;
    std::ifstream toParse;
    
public:
    
    LemonScriptCompiler(const std::string &toParseFileName, LemonScriptState *stateParam) : state(stateParam), toParse(stateParam->getSearchPath() + toParseFileName) {
        
        // Check that the file exists
        if(!toParse.good()) {
            throw "File not found: " + toParseFileName;
        }
        
        // THIS DOES ALL THE PARSING / COMPILATION
        rootSequence = new SequentialCommand(1, state, ParsingUtils::readWholeStream(toParse));
        
        if(rootSequence->getSequenceCount() == 0) {
            printf("Warning: empty file provided to Lemon Script\n");
        }
    }
    
    bool PeriodicUpdate() {
        if(!isDone) {
            isDone = rootSequence->Update();
        }
        return isDone;
    }
  
    ~LemonScriptCompiler() {
        delete rootSequence;
    }
};

#endif /* LemonScriptCompiler_hpp */
