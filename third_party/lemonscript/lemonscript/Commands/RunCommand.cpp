//
//  RunCommand.cpp
//  lemonscript
//
//  Created by Donald Pinckney on 2/21/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#include "RunCommand.h"

#include "LemonScriptCompiler.h"
#include "ParsingUtils.h"

lemonscript::RunCommand::RunCommand(int l, LemonScriptState *s, const std::string &commandString) : Command(l, s) {
    
    const std::string runDelim = "RUN";
    size_t runLoc = commandString.find(runDelim);
    size_t endOfRunLoc = runLoc + runDelim.length();
    
    std::string runFileName = commandString.substr(endOfRunLoc);
    runFileName = ParsingUtils::trimWhitespace(runFileName);
    
    s->pushScope();
    runCompiler = new lemonscript::LemonScriptCompiler(runFileName, s);
    runScope = s->getScope();
    s->popScope();
    
}

lemonscript::RunCommand::~RunCommand() {
    delete runCompiler;
}


bool lemonscript::RunCommand::Update() {
    LemonScriptSymbolTableStack currentScope = savedState->getScope();
    
    savedState->restoreScope(runScope);
    bool retVal = runCompiler->PeriodicUpdate();
    
    savedState->restoreScope(currentScope);
    
    return retVal;
}
