//
//  WhileAlsoCommand.hpp
//  FiniteStateMachine
//
//  Created by Donald Pinckney on 12/30/15.
//  Copyright © 2015 Donald Pinckney. All rights reserved.
//

#ifndef WhileAlsoCommand_hpp
#define WhileAlsoCommand_hpp

#include <stdio.h>

#include "SequentialCommand.h"
#include "LemonScriptSymbolTableStack.h"

#include "lemonscript.h"

class lemonscript::WhileAlsoCommand : public Command {
    
    SequentialCommand *whileCondition = NULL;
    SequentialCommand *alsoCommands = NULL;
    
    LemonScriptSymbolTableStack whileScope;
    LemonScriptSymbolTableStack alsoScope;
    
    bool isAlsoDone = false;
    
public:
    WhileAlsoCommand(int l, LemonScriptState *s, const std::string &commandString);
    virtual ~WhileAlsoCommand();
    
    bool Update();
    
};


#endif /* WhileAlsoCommand_hpp */
