//
//  SequentialCommand.cpp
//  lemonscript
//
//  Created by Donald Pinckney on 1/16/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#include "SequentialCommand.h"
#include "LemonScriptTokenizer.h"

#include "CppCommand.h"
#include "CompleteAnyCommand.h"
#include "CompleteCommand.h"
#include "SetCommand.h"
#include "DefCommand.h"
#include "ImportCommand.h"
#include "RunCommand.h"
#include "CommandFromToken.h"
#include "ParsingUtils.h"

#include <stdio.h>

void printTok(const std::string &tok, TokenType tk, int lineNum) {
    printf("===== TOKEN =====\nToken type = %d, lineNum = %d, tok = \n%s\n\n", tk, lineNum, tok.c_str());
}

lemonscript::SequentialCommand::SequentialCommand(int l, LemonScriptState *state, const std::string &sequenceString, bool explicitSequence, const std::string &prefixString) : Command(l, state) {
    

    std::string seqBody = sequenceString;
    // Optionally parse out SEQUENCE:
    isExplicit = explicitSequence;
    
    if(explicitSequence) {
        const std::string seqDelim = "SEQUENCE:\n";
        size_t seqLoc = sequenceString.find(seqDelim);
        
        size_t endOfSeqLoc = seqLoc + seqDelim.length();
        
        // Get the sequence body
        seqBody = sequenceString.substr(endOfSeqLoc);
        
        // Un-indent them before parsing
        seqBody = ParsingUtils::decreaseIndent(seqBody);
    }
    
    seqBody = prefixString + "\n" + seqBody;
    LemonScriptTokenizer tokenizer(seqBody);
    
    
    std::string token;
    TokenType type;
    int lineNum;
    
    if(isExplicit) {
        state->pushScope();
    }
    
    _hasExternalCode = false;
    while(true) {
        std::tie(token, type, lineNum) = tokenizer.nextToken();
        if(type == NOT_A_TOKEN) {
            break;
        }
                
        Command *command = lemonscript::commandFromToken(token, type, state, lineNum);
        sequence.push_back(command);
        
        _hasExternalCode = _hasExternalCode || command->HasExternalCode();
    }
    
    if(isExplicit) {
        sequenceScope = state->getScope();
        state->popScope();
    }
}

lemonscript::SequentialCommand::~SequentialCommand() {
    for (auto it = sequence.begin(); it != sequence.end(); ++it) {
        delete *it;
    }
}

bool lemonscript::SequentialCommand::Update() {
    LemonScriptSymbolTableStack currentScope = savedState->getScope();
    
    if(isExplicit) {
        savedState->restoreScope(sequenceScope);
    }
    
    bool done = updateSingleCommand() || fastForward();
    
    if(isExplicit) {
        savedState->restoreScope(currentScope);
    }
    
    return done;
}

// Returns true if the last command in the sequence just finished.
bool lemonscript::SequentialCommand::updateSingleCommand() {
    if(sequence.size() == 0) {
        return true;
    }
    
    Command *currentCommand = sequence[currentIndex];
    bool isDone = currentCommand->Update();
    
    // If the last command just finished, then we are done
    if(isDone && currentIndex == sequence.size() - 1) {
        return true;
    } else if(isDone) {
        // If a command other than the last finished, go to next command
        currentIndex++;
    }
    
    return false;
}

// Return true iff: we have already finished all the commands, or we just finished all the remaining commands by fast forwarding.
bool lemonscript::SequentialCommand::fastForward() {
    while (currentIndex < sequence.size() && sequence[currentIndex]->HasExternalCode() == false) {
        sequence[currentIndex]->fastForward();
        currentIndex++;
    }
    
    if(currentIndex < sequence.size()) {
        return sequence[currentIndex]->fastForward();
    } else {
        return true;
    }
}
