//
//  WhileAlsoCommand.cpp
//  FiniteStateMachine
//
//  Created by Donald Pinckney on 12/30/15.
//  Copyright © 2015 Donald Pinckney. All rights reserved.
//

#include "WhileAlsoCommand.h"

#include <iostream>

#include "ParsingUtils.h"

lemonscript::WhileAlsoCommand::WhileAlsoCommand(int l, LemonScriptState *s, const std::string &commandString) : Command(l, s) {
    const std::string whileDelim = "WHILE:\n";
    size_t whileLoc = commandString.find(whileDelim);
    size_t endOfWhileLoc = whileLoc + whileDelim.length();
    
    const std::string alsoDelim = "\nALSO:\n";
    size_t alsoLoc = commandString.find(alsoDelim);
    size_t endOfAlsoLoc = alsoLoc + alsoDelim.length();
    
    // Get the while and also bodies
    std::string whileBody = commandString.substr(endOfWhileLoc, alsoLoc - endOfWhileLoc);
    std::string alsoBody = commandString.substr(endOfAlsoLoc);
    
    // Un-indent them before parsing
    whileBody = ParsingUtils::decreaseIndent(whileBody);
    alsoBody = ParsingUtils::decreaseIndent(alsoBody);
    
    // Parse the bodies
    //TODO: Fix line number parameters.
    
    s->pushScope();
    whileCondition = new SequentialCommand(l, s, whileBody);
    whileScope = s->getScope();
    s->popScope();
    
    
    s->pushScope();
    alsoCommands = new SequentialCommand(l, s, alsoBody);
    alsoScope = s->getScope();
    s->popScope();
}

lemonscript::WhileAlsoCommand::~WhileAlsoCommand() {
    delete whileCondition;
    delete alsoCommands;
}


bool lemonscript::WhileAlsoCommand::Update() {
    
    LemonScriptSymbolTableStack currentScope = savedState->getScope();
    
    savedState->restoreScope(whileScope);
    bool conditionDone = whileCondition->Update();
    
    if(!isAlsoDone) {
        savedState->restoreScope(alsoScope);
        isAlsoDone = alsoCommands->Update();
    }
    
    savedState->restoreScope(currentScope);
    
    if(conditionDone) {
        return true;
    }
    
    return false;
}