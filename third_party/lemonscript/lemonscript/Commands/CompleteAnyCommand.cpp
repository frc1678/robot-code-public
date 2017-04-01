//
//  CompleteAnyToken.cpp
//  lemonscript
//
//  Created by Donald Pinckney on 1/16/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#include "CompleteAnyCommand.h"

#include "ParsingUtils.h"

lemonscript::CompleteAnyCommand::CompleteAnyCommand(int l, LemonScriptState *s, const std::string &commandString) : Command(l, s) {
    const std::string anyDelim = "COMPLETE ANY:\n";
    size_t anyLoc = commandString.find(anyDelim);
    size_t endOfAnyLoc = anyLoc + anyDelim.length();
    
    // Get the any body
    std::string anyBody = commandString.substr(endOfAnyLoc);
    
    // Un-indent them before parsing
    anyBody = ParsingUtils::decreaseIndent(anyBody);
    
    // Parse the bodies
    //TODO: Fix line number parameter.
    
    s->pushScope();
    anyCommands = new SimultaneousCommand(l, s, anyBody);
    anyScope = s->getScope();
    s->popScope();
    
    _hasExternalCode = anyCommands->HasExternalCode();
}

lemonscript::CompleteAnyCommand::~CompleteAnyCommand() {
    delete anyCommands;
}

bool lemonscript::CompleteAnyCommand::Update() {
    
    LemonScriptSymbolTableStack currentScope = savedState->getScope();
    
    savedState->restoreScope(anyScope);
    anyCommands->Update();
    
    savedState->restoreScope(currentScope);
    
    return anyCommands->getState() == SimultaneousCommmandState::AnyRequiredComplete || anyCommands->getState() == SimultaneousCommmandState::AllRequiredComplete;
}

bool lemonscript::CompleteAnyCommand::fastForward() {
    if(HasExternalCode()) {
        return anyCommands->getState() == SimultaneousCommmandState::AnyRequiredComplete || anyCommands->getState() == SimultaneousCommmandState::AllRequiredComplete;
    }
    
    while (Update() == false) { };
    
    return true;
}
