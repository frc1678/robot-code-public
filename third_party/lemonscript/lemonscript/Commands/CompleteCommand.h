//
//  CompleteCommand.hpp
//  FiniteStateMachine
//
//  Created by Donald Pinckney on 1/16/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#ifndef CompleteCommand_hpp
#define CompleteCommand_hpp

#include <stdio.h>

#include "SimultaneousCommand.h"
#include "LemonScriptSymbolTableStack.h"

#include "lemonscript.h"

class lemonscript::CompleteCommand : public Command {
    
    SimultaneousCommand *allCommands = NULL;
    
    LemonScriptSymbolTableStack allScope;
    
public:
    CompleteCommand(int l, LemonScriptState *s, const std::string &commandString);
    virtual ~CompleteCommand();
    
    bool Update();
    
};

#endif /* CompleteCommand_hpp */
