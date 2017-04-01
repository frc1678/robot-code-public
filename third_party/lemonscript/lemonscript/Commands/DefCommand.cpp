//
//  DefCommand.cpp
//  lemonscript
//
//  Created by Donald Pinckney on 2/17/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//


#include <iostream>
#include <string.h>

#include "DefCommand.h"
#include "ParsingUtils.h"


lemonscript::DefCommand::DefCommand(int l, LemonScriptState *s, const std::string &commandString) : Command(l, s) {
    state = s;
    _hasExternalCode = false;
    
    size_t equalsPos = commandString.find("=");
    if(equalsPos == std::string::npos) {
        throw "Invalid syntax: No '=' found in DEF command";
    }
    
    size_t DEFPos = commandString.find("DEF");
    if(DEFPos == std::string::npos) {
        throw "Invalid syntax: No 'DEF' found in DEF command";
    }
    size_t startLhsPos = DEFPos + 3;
    
    std::string lhs = ParsingUtils::trimWhitespace(commandString.substr(startLhsPos,  equalsPos - startLhsPos));
    std::string rhs = ParsingUtils::trimWhitespace(commandString.substr(equalsPos + 1));
    
    rhsExpression = new lemonscript_expressions::Expression(rhs, s, l);
    type = rhsExpression->getType();
    
    int zero;
    bzero(&zero, sizeof(int));
    
    state->declareVariable(l, lhs, rhsExpression->getType(), &zero);
    variableAddress = state->addressOfVariable(lhs);
}

lemonscript::DefCommand::~DefCommand() {
    delete rhsExpression;
}

bool lemonscript::DefCommand::Update() {
    rhsExpression->getValue(variableAddress);
    return true;
}

bool lemonscript::DefCommand::fastForward() {
    return Update();
}
