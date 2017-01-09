//
//  SequentialCommand.cpp
//  FiniteStateMachine
//
//  Created by Donald Pinckney on 1/16/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#include "SequentialCommand.h"
#include "LemonScriptTokenizer.h"

#include "CppCommand.h"
#include "WhileAlsoCommand.h"
#include "CompleteAnyCommand.h"
#include "CompleteCommand.h"
#include "SetCommand.h"
#include "DefCommand.h"
#include "ImportCommand.h"
#include "RunCommand.h"
#include "CommandFromToken.h"

#include <stdio.h>

void printTok(const std::string &tok, TokenType tk, int lineNum) {
    printf("===== TOKEN =====\nToken type = %d, lineNum = %d, tok = \n%s\n\n", tk, lineNum, tok.c_str());
}

lemonscript::SequentialCommand::SequentialCommand(int l, LemonScriptState *state, const std::string &sequenceString) : Command(l, state) {
    LemonScriptTokenizer tokenizer(sequenceString);
    
    
    std::string token;
    TokenType type;
    int lineNum;
    
    while(true) {
        std::tie(token, type, lineNum) = tokenizer.nextToken();
        if(type == NOT_A_TOKEN) {
            break;
        }
        
//        printTok(token, type, lineNum);
        
        Command *command = lemonscript::commandFromToken(token, type, state, lineNum);
        
        sequence.push_back(command);
    }
}

lemonscript::SequentialCommand::~SequentialCommand() {
    for (auto it = sequence.begin(); it != sequence.end(); ++it) {
        delete *it;
    }
}

bool lemonscript::SequentialCommand::Update() {
    if(sequence.size() == 0) {
        return true;
    }
    
    Command *currentCommand = sequence[currentIndex];
    bool isDone = currentCommand->Update();
    
    // If the last command just finished, then we are done
    if(isDone && static_cast<size_t>(currentIndex) == sequence.size() - 1) {
        return true;
    } else if(isDone) {
        // If a command other than the last finished, go to next command
        currentIndex++;
    }
    
    return false;
}
