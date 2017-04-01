//
//  SequentialCommand.hpp
//  lemonscript
//
//  Created by Donald Pinckney on 1/16/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#ifndef SequentialCommand_hpp
#define SequentialCommand_hpp

#include <stdio.h>
#include <vector>

#include "Command.h"

#include "lemonscript.h"

class lemonscript::SequentialCommand : public Command {

    std::vector<Command *> sequence;
    size_t currentIndex = 0;
    bool isExplicit;
    LemonScriptSymbolTableStack sequenceScope;
    bool updateSingleCommand();
    
public:
    SequentialCommand(int l, LemonScriptState *s, const std::string &sequenceString, bool explicitSequence, const std::string &prefixString);
    virtual ~SequentialCommand();

    bool Update();
    bool fastForward();
    size_t getSequenceCount() { return sequence.size(); }
};

#endif /* SequentialCommand_hpp */
