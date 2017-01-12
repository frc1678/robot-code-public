//
//  SequentialCommand.hpp
//  FiniteStateMachine
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
    int currentIndex = 0;
public:
    SequentialCommand(int l, LemonScriptState *s, const std::string &sequenceString);
    virtual ~SequentialCommand();

    bool Update();
    size_t getSequenceCount() { return sequence.size(); }
};

#endif /* SequentialCommand_hpp */
