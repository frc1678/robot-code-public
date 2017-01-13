//
//  OptionalCommand.hpp
//  lemonscript
//
//  Created by Donald Pinckney on 8/25/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#ifndef OptionalCommand_hpp
#define OptionalCommand_hpp

#include <stdio.h>

#include "Command.h"
#include "lemonscript.h"

class lemonscript::OptionalCommand : public Command {
    
    Command *command;
    bool isCommandComplete;
    
public:
    OptionalCommand(int l, LemonScriptState *s, const std::string &optionalCommandString);
    virtual ~OptionalCommand();
    
    bool Update();
};

#endif /* OptionalCommand_hpp */
