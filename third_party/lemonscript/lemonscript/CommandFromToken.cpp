//
//  CommandFromToken.cpp
//  lemonscript
//
//  Created by Donald Pinckney on 8/26/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#include "CommandFromToken.h"

#include "CppCommand.h"
#include "CompleteAnyCommand.h"
#include "CompleteCommand.h"
#include "SetCommand.h"
#include "DefCommand.h"
#include "ImportCommand.h"
#include "RunCommand.h"
#include "OptionalCommand.h"


lemonscript::Command *lemonscript::commandFromToken(const std::string &token, TokenType type, lemonscript::LemonScriptState *state, int lineNum) {
    Command *command;
    if(type == CppToken) {
        command = new CppCommand(lineNum, state, token);
    } else if(type == CompleteAnyToken) {
        command = new CompleteAnyCommand(lineNum, state, token);
    } else if(type == CompleteToken) {
        command = new CompleteCommand(lineNum, state, token);
    } else if(type == SetToken) {
        command = new SetCommand(lineNum, state, token);
    } else if(type == DefToken) {
        command = new DefCommand(lineNum, state, token);
    } else if(type == ImportToken) {
        command = new ImportCommand(lineNum, state, token);
    } else if(type == RunToken) {
        command = new RunCommand(lineNum, state, token);
    } else if(type == OptionalCommandToken) {
        command = new OptionalCommand(lineNum, state, token);
    } else if(type == SequenceToken) {
        command = new SequentialCommand(lineNum, state, token, true);
    }
    else {
        throw "Unknow token type: " + std::to_string(type);
    }
    
    return command;
}
