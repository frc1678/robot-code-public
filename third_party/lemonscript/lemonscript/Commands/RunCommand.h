//
//  RunCommand.hpp
//  lemonscript
//
//  Created by Donald Pinckney on 2/21/16.
//  Copyright © 2016 Donald Pinckney. All rights reserved.
//

#ifndef RunCommand_hpp
#define RunCommand_hpp

#include <stdio.h>

#include "SequentialCommand.h"

#include "lemonscript.h"

class lemonscript::RunCommand : public Command {
    
    LemonScriptCompiler *runCompiler = NULL;
    LemonScriptSymbolTableStack runScope;
    
public:
    RunCommand(int l, LemonScriptState *s, const std::string &commandString);
    virtual ~RunCommand();
    
    bool Update();
    
};
#endif /* RunCommand_hpp */
