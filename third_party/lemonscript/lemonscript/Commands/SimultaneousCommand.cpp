//
//  SimultaneousCommand.cpp
//  lemonscript
//
//  Created by Donald Pinckney on 1/16/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#include "SimultaneousCommand.h"
#include "LemonScriptTokenizer.h"

#include "CppCommand.h"
#include "CompleteAnyCommand.h"
#include "CompleteCommand.h"
#include "SetCommand.h"
#include "DefCommand.h"
#include "ImportCommand.h"
#include "RunCommand.h"
#include "CommandFromToken.h"
#include "OptionalCommand.h"

lemonscript::SimultaneousCommand::SimultaneousCommand(int l, LemonScriptState *state, const std::string &sequenceString) : Command(l, state) {
    LemonScriptTokenizer tokenizer(sequenceString);
    
    
    std::string token;
    TokenType type;
    int lineNum;
    
    while(true) {
        std::tie(token, type, lineNum) = tokenizer.nextToken();
        if(type == NOT_A_TOKEN) {
            break;
        }
        
        Command *command = lemonscript::commandFromToken(token, type, state, lineNum);
        commands.push_back(command);
        
        _hasExternalCode = _hasExternalCode || command->HasExternalCode();
    }
    
    doneCommands = std::vector<bool>(commands.size(), false);
}

lemonscript::SimultaneousCommand::~SimultaneousCommand() {
    for (auto it = commands.begin(); it != commands.end(); ++it) {
        delete *it;
    }
}


bool lemonscript::SimultaneousCommand::Update() {
    
    size_t len = commands.size();
    bool allDone = true;
    bool anyDone = false;
    for (size_t i = 0; i < len; i++) {
        Command *command = commands[i];
        bool alreadyDone = doneCommands[i];
        bool isOptional = dynamic_cast<lemonscript::OptionalCommand *>(command) != nullptr;
        
        if(alreadyDone && !isOptional) {
            anyDone = true;
            continue;
        }
        
        bool done = command->Update();
        doneCommands[i] = done;
        if(done && !isOptional) {
            anyDone = true;
        }
        if(!done) {
            allDone = false;
        }
    }
    
    if(allDone) {
        state = SimultaneousCommmandState::AllRequiredComplete;
    } else if(anyDone) {
        state = SimultaneousCommmandState::AnyRequiredComplete;
    }
    
    return allDone;
}

bool lemonscript::SimultaneousCommand::fastForward() {
    throw "Fast forward called on simultaneous command!";
}
