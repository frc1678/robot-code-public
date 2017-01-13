//
//  OptionalCommand.cpp
//  lemonscript
//
//  Created by Donald Pinckney on 8/25/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#include "OptionalCommand.h"

#include "ParsingUtils.h"
#include "CommandFromToken.h"

lemonscript::OptionalCommand::OptionalCommand(int l, lemonscript::LemonScriptState *s, const std::string &optionalCommandString) : lemonscript::Command(l, s), isCommandComplete(false)  {
    
    const std::string optionalDelim = "?";
    size_t optionalLoc = optionalCommandString.find(optionalDelim);
    size_t endOfOptionalLoc = optionalLoc + optionalDelim.length();
    
    std::string requiredCommandString = optionalCommandString.substr(endOfOptionalLoc);
    requiredCommandString = ParsingUtils::trimWhitespace(requiredCommandString);
    
    LemonScriptTokenizer tokenizer(requiredCommandString);
    
    std::string token;
    TokenType type;
    int lineNum;
    std::tie(token, type, lineNum) = tokenizer.nextToken();
    
#warning TODO: Fix line numbers!
    
    command = lemonscript::commandFromToken(token, type, s, lineNum);
}

lemonscript::OptionalCommand::~OptionalCommand() {
    delete command;
}

bool lemonscript::OptionalCommand::Update() {
    if(!isCommandComplete) {
        isCommandComplete = command->Update();
    }
    
    return true;
}
