//
//  SetCommand.hpp
//  lemonscript
//
//  Created by Donald Pinckney on 2/6/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#ifndef SetCommand_hpp
#define SetCommand_hpp

#include <stdio.h>
#include "Command.h"

#include "lemonscript.h"
#include "Expression.h"
#include "AvailableCppCommandDeclaration.h"
#include "LemonScriptState.h"

class lemonscript::SetCommand : public lemonscript::Command {
    
    lemonscript_expressions::Expression *rhsExpression;
    DataType type;
    void *variableAddress;
    lemonscript::LemonScriptState *state;
    
public:
    SetCommand(int l, LemonScriptState *s, const std::string &commandString);
    virtual ~SetCommand();
    
    bool Update();
    bool fastForward();

};


#endif /* SetCommand_hpp */
