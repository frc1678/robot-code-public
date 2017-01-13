//
//  DefCommand.hpp
//  lemonscript
//
//  Created by Donald Pinckney on 2/17/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#ifndef DefCommand_hpp
#define DefCommand_hpp

#include <stdio.h>

#include <stdio.h>
#include "Command.h"

#include "lemonscript.h"
#include "Expression.h"
#include "AvailableCppCommandDeclaration.h"
#include "LemonScriptState.h"

class lemonscript::DefCommand : public lemonscript::Command {
    
    lemonscript_expressions::Expression *rhsExpression;
    DataType type;
    void *variableAddress;
    lemonscript::LemonScriptState *state;
    
    
public:
    DefCommand(int l, LemonScriptState *s, const std::string &commandString);
    virtual ~DefCommand();
    
    bool Update();
    
};

#endif /* DefCommand_hpp */
