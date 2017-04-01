//
//  SetCommand.cpp
//  lemonscript
//
//  Created by Donald Pinckney on 2/6/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#include <iostream>
#include <string.h>

#include "SetCommand.h"
#include "ParsingUtils.h"


lemonscript::SetCommand::SetCommand(int l, LemonScriptState *s, const std::string &commandString) : Command(l, s) {
    state = s;
    _hasExternalCode = false;
    
    size_t equalsPos = commandString.find("=");
    if(equalsPos == std::string::npos) {
        throw "Invalid syntax: No '=' found in SET command";
    }
    
    size_t SETPos = commandString.find("SET");
    if(SETPos == std::string::npos) {
        throw "Invalid syntax: No 'SET' found in SET command";
    }
    size_t startLhsPos = SETPos + 3;
    
    std::string lhs = ParsingUtils::trimWhitespace(commandString.substr(startLhsPos,  equalsPos - startLhsPos));
    std::string rhs = ParsingUtils::trimWhitespace(commandString.substr(equalsPos + 1));
    
    rhsExpression = new lemonscript_expressions::Expression(rhs, s, l);
    type = rhsExpression->getType();
    
    int zero;
    bzero(&zero, sizeof(int));
    
    
    void *currentAddress = state->addressOfVariable(lhs);
    if(currentAddress == NULL) {
        throw "Line " + std::to_string(l) + ":\nUndeclared variable: " + lhs;
    } else {
        DataType existingType = state->typeOfVariable(lhs);
        if(existingType != type) {
            throw "Line " + std::to_string(l) + ":\nType error: can not assign type " + dataTypeDescription(type) + " to variable " + lhs + " of type " + dataTypeDescription(existingType);
        }
        
        variableAddress = currentAddress;
    }
}

lemonscript::SetCommand::~SetCommand() {
    delete rhsExpression;
}

bool lemonscript::SetCommand::Update() {
    rhsExpression->getValue(variableAddress);
    return true;
}

bool lemonscript::SetCommand::fastForward() {
    return Update();
}
